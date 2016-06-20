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

#ifndef PARABOLIC_RAMP_H
#define PARABOLIC_RAMP_H

#include <math.h>
#include "paraboliccommon.h"

namespace ParabolicRampInternal {

/** @file ParabolicRamp.h
 * @brief Functions for optimal acceleration-bounded trajectories.
 */

/** @brief Stores optimal trajectores for an acceleration and
 * velocity-bounded 1D system.
 *
 * Initialize the members x0 (start position), x1 (end position),
 * dx0 (start velocity), and dx1 (end velocity) before calling the
 * SolveX functions.
 */
class ParabolicRamp1D
{
public:
    ParabolicRamp1D() : x0(0), dx0(0), x1(0), dx1(0), tswitch1(0), tswitch2(0), ttotal(0), a1(0), v(0), a2(0) {
    }
    /// New function added on 2013/04/24 for setting a ramp with desired position, velocity and time duration
    void SetPosVelTime(Real _x0,Real _dx0,Real _x1,Real _dx1,Real t);

    /// Sets the ramp to a constant function for time t
    void SetConstant(Real x,Real t=0);
    /// Sets the ramp to a linear function from x0 to x1 with time t.
    void SetLinear(Real x0,Real x1,Real t);
    /// Solves for minimum time given acceleration and velocity bounds
    bool SolveMinTime(Real amax,Real vmax);
    /// Solves for minimum time given acceleration and velocity bounds, min time
    bool SolveMinTime2(Real amax,Real vmax,Real tLowerBound);
    /// Solves for minimum acceleration given end time and velocity bounds
    bool SolveMinAccel(Real endTime,Real vmax);

    /// Solves for minimum acceleration given end time and velocity bounds with a new method. \author Puttichai Lertkultanon
    bool SolveMinAccel(Real endTime, Real vmax, Real amax); ////////Puttichai
    /// Solves for a value of tswitch1 (used in SolveMinAccel)
    bool SolveForTSwitch1(Real A, Real B, Real endTime, Real l, Real u); ////////Puttichai

    /// Solves for the minimum-time braking trajectory starting from x0,dx0
    void SolveBraking(Real amax);
    /// Solves for the ramp given max the exact time
    bool SolveFixedTime(Real amax,Real vmax,Real endTime);
    /// solves for the ramp given fixed switch times and end time
    bool SolveFixedSwitchTime(Real amax,Real vmax);
    /// solves for the ramp given fixed tswitch1 and (ttotal-tswitch2). ttotal and tswitch2 will be solved for.
    bool SolveFixedAccelSwitchTime(Real amax,Real vmax, Real deltaswitch1, Real deltaswitch3);
    /// Evaluates the trajectory
    Real Evaluate(Real t) const;
    /// Evaluates the derivative of the trajectory
    Real Derivative(Real t) const;
    /// Evaluates the second derivative of the trajectory
    Real Accel(Real t) const;
    /// Returns the time at which x1 is reached
    Real EndTime() const {
        return ttotal;
    }
    /// Scales time to slow down (value > 1) or speed up (< 1) the trajectory
    void Dilate(Real timeScale);
    /// Trims off the front [0,tcut] of the trajectory
    void TrimFront(Real tcut);
    /// Trims off the front [T-tcut,T] of the trajectory
    void TrimBack(Real tcut);
    /// Returns the x bounds on the path
    void Bounds(Real& xmin,Real& xmax) const;
    /// Returns the x bounds for the given time interval
    void Bounds(Real ta,Real tb,Real& xmin,Real& xmax) const;
    /// Returns the v bounds on the path
    void DerivBounds(Real& vmin,Real& vmax) const;
    /// Returns the v bounds for the given time interval
    void DerivBounds(Real ta,Real tb,Real& vmin,Real& vmax) const;
    /// Sanity check
    bool IsValid() const;

    /// Input
    Real x0,dx0;
    Real x1,dx1;

    /// Calculated upon SolveX
    Real tswitch1,tswitch2;  //time to switch between ramp/flat/ramp
    Real ttotal;
    Real a1,v,a2;   // accel of first ramp, velocity of linear section, accel of second ramp
};

/** @brief Solves for optimal trajectores for a velocity-bounded ND system.
 *
 * Methods are essentially the same as for ParabolicRamp1D.
 */
class ParabolicRampND
{
public:
    ParabolicRampND() : endTime(0), constraintchecked(0), modified(0) {
    }
    /// New function added on 2013/04/24 for setting a ramp with desired position, velocity and time duration
    void SetPosVelTime(const Vector& _x0, const Vector& _dx0, const Vector& _x1, const Vector& _dx1,Real t);

    void SetConstant(const Vector& x,Real t=0);
    void SetLinear(const Vector& x0,const Vector& x1,Real t);
    bool SolveMinTimeLinear(const Vector& amax,const Vector& vmax);
    bool SolveMinTime(const Vector& amax,const Vector& vmax);
    bool SolveMinAccel(const Vector& vmax,Real time);
    bool SolveMinAccelLinear(const Vector& vmax,Real time);
    void SolveBraking(const Vector& amax);
    void Evaluate(Real t,Vector& x) const;
    void Derivative(Real t,Vector& dx) const;
    void Accel(Real t,Vector& ddx) const;
    void Output(Real dt,std::vector<Vector>& path) const;
    void Dilate(Real timeScale);
    void TrimFront(Real tcut);
    void TrimBack(Real tcut);
    void Bounds(Vector& xmin,Vector& xmax) const;
    void Bounds(Real ta,Real tb,Vector& xmin,Vector& xmax) const;
    void DerivBounds(Vector& vmin,Vector& vmax) const;
    void DerivBounds(Real ta,Real tb,Vector& vmin,Vector& vmax) const;
    bool IsValid() const;

    /// Input
    Vector x0,dx0;
    Vector x1,dx1;

    /// Calculated upon SolveX
    Real endTime;
    std::vector<ParabolicRamp1D> ramps;

    mutable int constraintchecked; ///< 0 if collision hasn't been checked yet, otherwise 1
    mutable int modified; ///< 0 if has not been modified by the merging operation, otherwise 1

};

/// Computes a min-time ramp from (x0,v0) to (x1,v1) under the given
/// acceleration, velocity, and x bounds.  Returns true if successful.
bool SolveMinTimeBounded(Real x0,Real v0,Real x1,Real v1, Real amax,Real vmax,Real xmin,Real xmax, ParabolicRamp1D& ramp);

/// Computes a sequence of up to three ramps connecting (x0,v0) to (x1,v1)
/// in minimum-acceleration fashion with a fixed end time, under the given
/// velocity and x bounds.  Returns true if successful.
bool SolveMinAccelBounded(Real x0,Real v0,Real x1,Real v1, Real endTime,Real vmax,Real xmin,Real xmax, std::vector<ParabolicRamp1D>& ramps);

bool SolveMinAccelBounded(Real x0,Real v0,Real x1,Real v1, Real endTime,Real vmax, Real amax, Real xmin,Real xmax, std::vector<ParabolicRamp1D>& ramps); ////////Puttichai

bool SolveMaxAccelBounded(Real x0,Real v0,Real x1,Real v1, Real endTime, Real amax, Real vmax,Real xmin,Real xmax, std::vector<ParabolicRamp1D>& ramps);

/// Vector version of above.
/// Returns the time of the minimum time trajectory, or -1 on failure
/// \param multidofinterp if true, will always force the max acceleration of the robot when retiming rather than using lesser acceleration whenever possible
Real SolveMinTimeBounded(const Vector& x0,const Vector& v0,const Vector& x1,const Vector& v1, const Vector& amax,const Vector& vmax,const Vector& xmin,const Vector& xmax, std::vector<std::vector<ParabolicRamp1D> >& ramps, int multidofinterp);

Real SolveMinTimeBounded2(const Vector& x0,const Vector& v0,const Vector& x1,const Vector& v1, const Vector& amax,const Vector& vmax,const Vector& xmin,const Vector& xmax, std::vector<std::vector<ParabolicRamp1D> >& ramps, int multidofinterp);////////Puttichai

/// Vector version of above.
/// Returns true if successful.
bool SolveMinAccelBounded(const Vector& x0,const Vector& v0,const Vector& x1,const Vector& v1, Real endTime,const Vector& vmax,const Vector& xmin,const Vector& xmax, std::vector<std::vector<ParabolicRamp1D> >& ramps);

bool SolveMinAccelBounded(const Vector& x0,const Vector& v0,const Vector& x1,const Vector& v1, Real endTime,const Vector& vmax,const Vector& amax, const Vector& xmin,const Vector& xmax, std::vector<std::vector<ParabolicRamp1D> >& ramps);////////Puttichai

/// if 0 - SolveMinAccelBounded, if 1 - SolveMaxAccelBounded, if 2 - all ramps have same switch points
bool SolveAccelBounded(const Vector& x0,const Vector& v0,const Vector& x1,const Vector& v1, Real endTime,const Vector& amax,const Vector& vmax,const Vector& xmin,const Vector& xmax, std::vector<std::vector<ParabolicRamp1D> >& ramps, int multidofinterp);

/// Combines an array of 1-d ramp sequences into a sequence of N-d ramps
//void CombineRamps(const std::vector<std::vector<ParabolicRamp1D> >& ramps,std::vector<ParabolicRampND>& ndramps);
//void CombineRamps(const std::vector<std::vector<ParabolicRamp1D> >& ramps,std::list<ParabolicRampND>& ndramps);

// T can be std::list<ParabolicRampND> or std::vector<ParabolicRampND>
template <typename T>
void CombineRamps(const std::vector<std::vector<ParabolicRamp1D> >& ramps, T& ndramps)
{
    PARABOLIC_RAMP_ASSERT(ndramps.size()==0);
    ndramps.clear();
    std::vector<std::vector<ParabolicRamp1D>::const_iterator> indices(ramps.size());
    for(size_t i=0; i<ramps.size(); i++) {
        PARABOLIC_RAMP_ASSERT(!ramps[i].empty());
        indices[i] = ramps[i].begin();
    }
    Real tmax = Inf; // the max possible time to create the new set of ramps. This is equivalent to the min of the max times of all the input ramps. If any of the input ramps have different timestamps, then throw an exception
    for(size_t idof = 0; idof < ramps.size(); ++idof) {
        Real tdofmax = 0;
        for(size_t iramp = 0; iramp < ramps[idof].size(); ++iramp) {
            tdofmax += ramps[idof][iramp].ttotal;
        }
        if( tmax > tdofmax ) {
            tmax = tdofmax;
        }
    }

    std::vector<Real> timeOffsets(ramps.size(),0);  //start time of current index
    Real t=0; // current time
    while(true) {
        //pick next ramp
        Real tnext=Inf;
        for(size_t i=0; i<ramps.size(); i++) {
            if(indices[i] != ramps[i].end()) {
                tnext = Min(tnext,timeOffsets[i]+indices[i]->ttotal);
            }
        }
        if(IsInf(tnext)) {
            // done
            break;
        }
        // have to clamp to the max
        if( tnext > tmax ) {
            PARABOLIC_RAMP_PLOG("tnext is greater than the max, so truncating. diff = %.15f", (tnext-tmax));
            tnext = tmax;
            if( tnext <= t ) {
                // just finish, there's nothing more to do...
                break;
            }
        }

        if(!(tnext > t || t == 0)) {
            PARABOLIC_RAMP_PLOG("CombineRamps: error finding next time step?\n");
            PARABOLIC_RAMP_PLOG("tnext = %g, t = %g, step = %d\n",tnext,t,ndramps.size());
            for(size_t k=0; k<ramps.size(); k++) {
                PARABOLIC_RAMP_PLOG("Ramp %d times: ",k);
                Real ttotal = 0.0;
                for(size_t j=0; j<ramps[k].size(); j++) {
                    PARABOLIC_RAMP_PLOG("%g ",ramps[k][j].ttotal);
                    ttotal += ramps[k][j].ttotal;
                }
                PARABOLIC_RAMP_PLOG(", total %g\n",ttotal);
            }
        }
        PARABOLIC_RAMP_ASSERT(tnext > t || t == 0);
        if(tnext == 0) {
            for(size_t i=0; i<ramps.size(); i++) {
                if(ramps[i].size()!=1) {
                    PARABOLICWARN("Warning, some entry has multiple zeroes?\n");
                    for(size_t j=0; j<ramps.size(); j++) {
                        PARABOLICWARN("Ramp %d: ",j);
                        for(size_t k=0; k<ramps[j].size(); k++) {
                            PARABOLICWARN("%g ",ramps[j][k].ttotal);
                        }
                    }
                }
                PARABOLIC_RAMP_ASSERT(ramps[i].size()==1);
            }
        }

        typename T::iterator itramp = ndramps.insert(ndramps.end(), ParabolicRampND());
        itramp->x0.resize(ramps.size());
        itramp->x1.resize(ramps.size());
        itramp->dx0.resize(ramps.size());
        itramp->dx1.resize(ramps.size());
        itramp->ramps.resize(ramps.size());
        itramp->endTime = tnext-t;
        for(size_t i=0; i<ramps.size(); i++) {
            if(indices[i] != ramps[i].end()) {
                ParabolicRamp1D iramp = *indices[i];
                if(indices[i] == --ramps[i].end() && FuzzyEquals(tnext-timeOffsets[i],indices[i]->ttotal,EpsilonT*0.1)) {
                    //don't trim back
                }
                else {
                    iramp.TrimBack((timeOffsets[i]-tnext)+indices[i]->ttotal);
                }
                iramp.TrimFront(t-timeOffsets[i]);
                Real oldTotal = iramp.ttotal;
                iramp.ttotal = itramp->endTime;
                if(iramp.tswitch1 > iramp.ttotal) {
                    iramp.tswitch1 = iramp.ttotal;
                }
                if(iramp.tswitch2 > iramp.ttotal) {
                    iramp.tswitch2 = iramp.ttotal;
                }
                if(!iramp.IsValid()) {
                    PARABOLICWARN("CombineRamps: Trimming caused ramp to become invalid\n");
                    PARABOLICWARN("Old total time %g, new total time %g\n",oldTotal,iramp.ttotal);
                }
                PARABOLIC_RAMP_ASSERT(iramp.IsValid());
                itramp->ramps[i] = iramp;
                itramp->x0[i] = iramp.x0;
                itramp->dx0[i] = iramp.dx0;
                itramp->x1[i] = iramp.x1;
                itramp->dx1[i] = iramp.dx1;
                if(FuzzyEquals(tnext,timeOffsets[i]+indices[i]->ttotal,EpsilonT*0.1)) {
                    timeOffsets[i] = tnext;
                    indices[i]++;
                }
                PARABOLIC_RAMP_ASSERT(itramp->ramps[i].ttotal == itramp->endTime);
            }
            else {
                //after the last segment, propagate a constant off the last ramp
                PARABOLIC_RAMP_ASSERT(!ramps[i].empty());
                itramp->x0[i] = ramps[i].back().x1;
                itramp->dx0[i] = ramps[i].back().dx1;
                //itramp->x1[i] = ramps[i].back().x1+ramps[i].back().dx1*(tnext-t);
                itramp->x1[i] = ramps[i].back().x1;
                itramp->dx1[i] = ramps[i].back().dx1;
                if(!FuzzyEquals(ramps[i].back().dx1*(tnext-t),0.0,EpsilonV)) {
                    PARABOLIC_RAMP_PLOG("CombineRamps: warning, propagating time %g distance %g off the back, vel %g\n",(tnext-t),ramps[i].back().dx1*(tnext-t),itramp->dx0[i]);

                    for(size_t k=0; k<ramps.size(); k++) {
                        PARABOLIC_RAMP_PLOG("Ramp %d times: ",k);
                        Real ttotal = 0.0;
                        for(size_t j=0; j<ramps[k].size(); j++) {
                            PARABOLIC_RAMP_PLOG("%g ",ramps[k][j].ttotal);
                            ttotal += ramps[k][j].ttotal;
                        }
                        PARABOLIC_RAMP_PLOG(", total %g\n",ttotal);
                    }
                }
                //set the 1D ramp manually
                itramp->ramps[i].x0 = itramp->x0[i];
                itramp->ramps[i].dx0 = itramp->dx0[i];
                itramp->ramps[i].x1 = itramp->x1[i];
                itramp->ramps[i].dx1 = itramp->dx1[i];
                itramp->ramps[i].ttotal = itramp->ramps[i].tswitch2 = (tnext-t);
                itramp->ramps[i].tswitch1 = 0;
                itramp->ramps[i].v = itramp->dx1[i];
                itramp->ramps[i].a1 = itramp->ramps[i].a2 = 0;
                if( !itramp->ramps[i].IsValid() ) {
                    // usually do to small epsilons...
                    PARABOLICWARN("combined ramp is invalid, so try solving for new variables to validate it\n");
                    if( !itramp->ramps[i].SolveMinAccel(tnext-t, 2*fabs(itramp->dx1[i])+0.1) ) {
                        PARABOLICWARN("could not solve for min accel!\n");
                        PARABOLIC_RAMP_ASSERT(0);
                    }
                }
                else {
                    PARABOLIC_RAMP_ASSERT(itramp->ramps[i].ttotal == itramp->endTime);
                }
                //PARABOLIC_RAMP_ASSERT(itramp->ramps[i].IsValid());

            }
        }
        PARABOLIC_RAMP_ASSERT(itramp->IsValid());
        if(ndramps.size() > 1) { //fix up endpoints
            typename T::iterator itprevramp = ---- ndramps.end();
            itramp->x0 = itprevramp->x1;
            itramp->dx0 = itprevramp->dx1;
            for(size_t i=0; i<itramp->ramps.size(); i++) {
                itramp->ramps[i].x0=itramp->x0[i];
                itramp->ramps[i].dx0=itramp->dx0[i];
            }
        }
        PARABOLIC_RAMP_ASSERT(itramp->IsValid());

        t = tnext;
        if(tnext == 0) { //all null ramps
            break;
        }
    }
    for(size_t i=0; i<ramps.size(); i++) {
        if(!FuzzyEquals(ramps[i].front().x0,ndramps.front().x0[i],EpsilonX)) {
            PARABOLIC_RAMP_PLOG("CombineRamps: Error: %d start %g != %g\n",i,ramps[i].front().x0,ndramps.front().x0[i]);
        }
        if(!FuzzyEquals(ramps[i].front().dx0,ndramps.front().dx0[i],EpsilonV)) {
            PARABOLIC_RAMP_PLOG("CombineRamps: Error: %d start %g != %g\n",i,ramps[i].front().dx0,ndramps.front().dx0[i]);
        }
        if(!FuzzyEquals(ramps[i].back().x1,ndramps.back().x1[i],EpsilonX)) {
            PARABOLIC_RAMP_PLOG("CombineRamps: Error: %d back %g != %g\n",i,ramps[i].back().x1,ndramps.back().x1[i]);
        }
        if(!FuzzyEquals(ramps[i].back().dx1,ndramps.back().dx1[i],EpsilonV)) {
            PARABOLIC_RAMP_PLOG("CombineRamps: Error: %d back %g != %g\n",i,ramps[i].back().dx1,ndramps.back().dx1[i]);
        }
        ndramps.front().x0[i] = ndramps.front().ramps[i].x0 = ramps[i].front().x0;
        ndramps.front().dx0[i] = ndramps.front().ramps[i].dx0 = ramps[i].front().dx0;
        ndramps.back().x1[i] = ndramps.back().ramps[i].x1 = ramps[i].back().x1;
        ndramps.back().dx1[i] = ndramps.back().ramps[i].dx1 = ramps[i].back().dx1;
    }
}

////////Puttichai
// The following two functions are added for SolveMinAccel
// Compute roots of a cubic polynomial. This functions is taken from OpenRAVE ik generator file.
static inline void FindPolyRoots3(Real rawcoeffs[4], Real rawroots[3], int& numroots) {
    using std::complex;
    if( rawcoeffs[0] == 0 ) {
        // solve with one reduced degree
        //%(reducedpolyroots)s(&rawcoeffs[1], &rawroots[0], numroots);
        numroots = 0;
        return;
    }
    const Real tol = 128.0*std::numeric_limits<Real>::epsilon();
    const Real tolsqrt = sqrt(std::numeric_limits<Real>::epsilon());
    complex<Real> coeffs[3];
    const int maxsteps = 110;
    for(int i = 0; i < 3; ++i) {
        coeffs[i] = complex<Real>(rawcoeffs[i+1]/rawcoeffs[0]);
    }
    complex<Real> roots[3];
    Real err[3];
    roots[0] = complex<Real>(1,0);
    roots[1] = complex<Real>(0.4,0.9); // any complex number not a root of unity works
    err[0] = 1.0;
    err[1] = 1.0;
    for(int i = 2; i < 3; ++i) {
        roots[i] = roots[i-1]*roots[1];
        err[i] = 1.0;
    }
    for(int step = 0; step < maxsteps; ++step) {
        bool changed = false;
        for(int i = 0; i < 3; ++i) {
            if ( err[i] >= tol ) {
                changed = true;
                // evaluate
                complex<Real> x = roots[i] + coeffs[0];
                for(int j = 1; j < 3; ++j) {
                    x = roots[i] * x + coeffs[j];
                }
                for(int j = 0; j < 3; ++j) {
                    if( i != j ) {
                        if( roots[i] != roots[j] ) {
                            x /= (roots[i] - roots[j]);
                        }
                    }
                }
                roots[i] -= x;
                err[i] = abs(x);
            }
        }
        if( !changed ) {
            break;
        }
    }

    numroots = 0;
    bool visited[3] = {false};
    for(int i = 0; i < 3; ++i) {
        if( !visited[i] ) {
            // might be a multiple root, in which case it will have more error than the other roots
            // find any neighboring roots, and take the average
            complex<Real> newroot=roots[i];
            int n = 1;
            for(int j = i+1; j < 3; ++j) {
                // care about error in real much more than imaginary
                if( std::abs(real(roots[i])-real(roots[j])) < tolsqrt && std::abs(imag(roots[i])-imag(roots[j])) < 0.002 ) {
                    newroot += roots[j];
                    n += 1;
                    visited[j] = true;
                }
            }
            if( n > 1 ) {
                newroot /= n;
            }
            // there are still cases where even the mean is not accurate enough, until a better
            // multi-root algorithm is used, need to use the sqrt
            if( fabs(imag(newroot)) < tolsqrt ) {
                rawroots[numroots++] = real(newroot);
            }
        }
    }
}

static inline void FindPolyRoots4(Real rawcoeffs[5], Real rawroots[4], int& numroots)
{
    using std::complex;
    if( rawcoeffs[0] == 0 ) {
        // solve with one reduced degree
        //%(reducedpolyroots)s(&rawcoeffs[1], &rawroots[0], numroots);
        Real newrawcoeffs[4] = {rawcoeffs[1], rawcoeffs[2], rawcoeffs[3], rawcoeffs[4]};
        FindPolyRoots3(newrawcoeffs, rawroots, numroots);
        // numroots = 0;
        return;
    }
    const Real tol = 128.0*std::numeric_limits<Real>::epsilon();
    const Real tolsqrt = sqrt(std::numeric_limits<Real>::epsilon());
    complex<Real> coeffs[4];
    const int maxsteps = 110;
    for(int i = 0; i < 4; ++i) {
        coeffs[i] = complex<Real>(rawcoeffs[i+1]/rawcoeffs[0]);
    }
    complex<Real> roots[4];
    Real err[4];
    roots[0] = complex<Real>(1,0);
    roots[1] = complex<Real>(0.4,0.9); // any complex number not a root of unity works
    err[0] = 1.0;
    err[1] = 1.0;
    for(int i = 2; i < 4; ++i) {
        roots[i] = roots[i-1]*roots[1];
        err[i] = 1.0;
    }
    for(int step = 0; step < maxsteps; ++step) {
        bool changed = false;
        for(int i = 0; i < 4; ++i) {
            if ( err[i] >= tol ) {
                changed = true;
                // evaluate
                complex<Real> x = roots[i] + coeffs[0];
                for(int j = 1; j < 4; ++j) {
                    x = roots[i] * x + coeffs[j];
                }
                for(int j = 0; j < 4; ++j) {
                    if( i != j ) {
                        if( roots[i] != roots[j] ) {
                            x /= (roots[i] - roots[j]);
                        }
                    }
                }
                roots[i] -= x;
                err[i] = abs(x);
            }
        }
        if( !changed ) {
            break;
        }
    }

    numroots = 0;
    bool visited[4] = {false};
    for(int i = 0; i < 4; ++i) {
        if( !visited[i] ) {
            // might be a multiple root, in which case it will have more error than the other roots
            // find any neighboring roots, and take the average
            complex<Real> newroot=roots[i];
            int n = 1;
            for(int j = i+1; j < 4; ++j) {
                // care about error in real much more than imaginary
                if( std::abs(real(roots[i])-real(roots[j])) < tolsqrt && std::abs(imag(roots[i])-imag(roots[j])) < 0.002 ) {
                    newroot += roots[j];
                    n += 1;
                    visited[j] = true;
                }
            }
            if( n > 1 ) {
                newroot /= n;
            }
            // there are still cases where even the mean is not accurate enough, until a better
            // multi-root algorithm is used, need to use the sqrt
            if( fabs(imag(newroot)) < tolsqrt ) {
                rawroots[numroots++] = real(newroot);
            }
        }
    }
}



} //namespace ParabolicRamp

#endif
