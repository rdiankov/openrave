// -*- coding: utf-8 -*-
// Copyright (C) 2016 Puttichai Lertkultanon & Rosen Diankov
//
// This program is free software: you can redistribute it and/or modify it under the terms of the
// GNU Lesser General Public License as published by the Free Software Foundation, either version 3
// of the License, or at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
// even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
#ifndef RAMP_OPTIM_INTERPOLATOR_H
#define RAMP_OPTIM_INTERPOLATOR_H

#include "ramp.h"

namespace OpenRAVE {

namespace RampOptimizerInternal {

class ParabolicInterpolator {
public:
    /*
       Given boundary values (initial & final position(s)/velocity(-ies)), compute a trajectory which
       interpolates those values. Functions which compute ND trajectories always put the final
       results in the form of a vector of RampND.
     */
    ParabolicInterpolator() {
    }
    ParabolicInterpolator(size_t ndof, int envid=0);
    ~ParabolicInterpolator() {
    }

    void Initialize(size_t ndof, int envid=0);

    /// ND Trajectory

    /**
       \brief Compute an ND trajectory which interpolates (x0Vect, 0) and (x1Vect, 0) while
       respecting velocity and acceleration limits. In this case the trajectory can be computed such
       that the path geometry does not change. Therefore, assuming that the initial path (the
       straight line connecting x0Vect and x1Vect) is valid, we do not need to check joint limits
       here.

       \param x0Vect initial position
       \param x1Vect final position
       \param vmVect velocity limits
       \param amVect acceleration limits
     */
    bool ComputeZeroVelNDTrajectory(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect, const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, std::vector<RampND>& rampndVectOut);

    /**
       \brief Compute an ND trajectory which interpolates (x0Vect, v0Vect) and (x1Vect, v1Vect)
       while respecting velocity and acceleration limits, as well as joint limits.

       Procedure: 1. Compute 1D trajectory for each DOF seperately. Each 1D trajectory may have
                     different duration. Denote the longest duration as tmax.

                  2a. If not tryHarder, stretch the duration of all trajectories to tmax. This step
                      may not always feasible due to inoperative time interval for each joint as
                      well as joint limits.

                  2b. If tryHarder, first compute the least upper bound of inoperative time
                      intervals of all joints, tbound. Then stretch the duration of all trajectories
                      to tbound. This step may not always feasible, however, due to joint limits
                      which have not been taken into account when calculating tbound.

       \param x0Vect initial position
       \param x1Vect final position
       \param v0Vect initial velocity
       \param v1Vect final velocity
       \param xminVect lower joint limits
       \param xmaxVect upper joint limits
       \param vmVect velocity limits
       \param amVect acceleration limits
       \params tryHarder
     */
    bool ComputeArbitraryVelNDTrajectory(const std::vector<dReal>&x0Vect, const std::vector<dReal>&x1Vect, const std::vector<dReal>&v0Vect, const std::vector<dReal>&v1Vect, const std::vector<dReal>&xminVect, const std::vector<dReal>&xmaxVect, const std::vector<dReal>&vmVect, const std::vector<dReal>&amVect, std::vector<RampND>&rampndOut, bool tryHarder);

    /**
       \brief This function is called from inside ComputeArbitraryVelNDTrajectory to stretch the
       duration of all the trajectories to some value t. If not tryHarder, t is equal to the longest
       trajectory duration. Otherwise, t will be calculated by taking into account inoperative time
       intervals of every joint.

       \param curvesVect the input vector of ParabolicCurves. This will also carry the resulting ParabolicCurves.
       \param vmVect velocity limts
       \param amVect acceleration limits
       \param maxIndex the index of the trajectory with the longest duration
       \param tryHarder
     */
    bool _RecomputeNDTrajectoryFixedDuration(std::vector<ParabolicCurve>& curvesVect, const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, size_t maxIndex, bool tryHarder);

    /**

     */
    bool ComputeNDTrajectoryFixedDuration(const std::vector<dReal>&x0Vect, const std::vector<dReal>&x1Vect, const std::vector<dReal>&v0Vect, const std::vector<dReal>&v1Vect, dReal duration, const std::vector<dReal>&xminVect, const std::vector<dReal>&xmaxVect, const std::vector<dReal>&vmVect, const std::vector<dReal>&amVect, std::vector<RampND>&rampndOut);

    /// 1D Trajectory

    /**
       \brief Compute the minimum-time 1D parabolic trajectory which interpolates (x0, v0) and (x1,
       v1) while respecting velocity and acceleration limits. Note, however, that joint limits are
       not taken into account here.

       We separate the computation into two parts, computing a trajectory without velocity limit and
       imposing velocity limit.

       \param x0 initial position
       \param x1 final position
       \param v0 initial velocity
       \param v1 final velocity
       \param vm velocity limit
       \param am acceleration limit
     */
    bool Compute1DTrajectory(dReal x0, dReal x1, dReal v0, dReal v1, dReal vm, dReal am, ParabolicCurve& curveOut, bool bCheck=true);

    /**
       \brief Impose the given joint limits to a 1D parabolic trajectory while maintaining the
       original duration of the trajectory. This function is only called from ND trajectory
       generation functions.

       Note: the algorithm is taken from Hauser's implementation of parabolic shortcut.
     */
    bool _ImposeJointLimitFixedDuration(ParabolicCurve& curve, dReal xmin, dReal xmax, dReal vm, dReal am, bool bCheck=true);

    /**
       \brief Stretch the paraboliccurve such that it ends at the given duration newDuration. This
       function internally calls Compute1DTrajectoryFixedDuration.
     */
    bool Stretch1DTrajectory(ParabolicCurve& curve, dReal vm, dReal am, dReal newDuration);

    /**
       \brief Compute a 1D minimum-acceleration parabolic trajectory interpolating (x0, v0) and (x1,
       v1) with the specified duration.
     */
    bool Compute1DTrajectoryFixedDuration(dReal x0, dReal x1, dReal v0, dReal v1, dReal vm, dReal am, dReal duration, ParabolicCurve& curveOut);

    /// Utilities

    /**
       \brief Calculate the least upper bound of the inoperative interval(s), t,of the given
       trajectory. The value t is such that stretching the trajectory with a duration of greater
       than or equal to t will always be feasible (considering only velocity and acceleration
       limits).
     */
    bool _CalculateLeastUpperBoundInoperativeTimeInterval(dReal x0, dReal x1, dReal v0, dReal v1, dReal vm, dReal am, dReal& t);

    /**
       \brief Solve for a switch time t0 \in [l, u]. For more detail, see comments in the
       implementation.
     */
    bool _SolveForT0(dReal A, dReal B, dReal t, dReal l, dReal u, dReal& t0);

    /**
       \brief Convert a vector of 1D trajectories (with equal durations; stored as ParabolicCurves)
       to a vector of RampNDs.

       During ParabolicCurves evaluation, some small discrepancies might occur (small but large
       enough to be caught by parabolicchecker), especially the acceleration values. So we
       re-compute the accelration values to try to minimize those discrepancies. However, we also
       need to make sure that our acceleration corrections are valid, i.e., the computed
       acceleration must be within bounds. Therefore, we also take amVect as an *optional* argument
       here. If given, we re-compute accelerations so as to minimize discrepancies. Otherwise, we
       use the acceleration values evaluated from curvesVectIn.
     */
    void _ConvertParabolicCurvesToRampNDs(const std::vector<ParabolicCurve>& curvesVectIn, std::vector<RampND>& rampndVectOut, const std::vector<dReal>& amVect=std::vector<dReal>());

    inline dReal SolveBrakeTime(dReal x, dReal v, dReal xbound) {
        dReal bt;
        bool res = SafeEqSolve(v, 2*(xbound - x), g_fRampEpsilon, 0, g_fRampInf, bt);
        if( !res ) {
            RAVELOG_VERBOSE_FORMAT("Cannot solve the brake time equation: %.15e*t - %.15e = 0 with t being in [0, inf)", v%(2*(xbound - x)));
            bt = 0;
        }
        return bt;
    }

    inline dReal SolveBrakeAccel(dReal x, dReal v, dReal xbound) {
        dReal ba;
        dReal coeff0 = 2*(xbound - x);
        dReal coeff1 = v*v;
        bool res = SafeEqSolve(coeff0, -coeff1, g_fRampEpsilon, -g_fRampInf, g_fRampInf, ba);
        if( !res ) {
            RAVELOG_VERBOSE_FORMAT("Cannot solve the brake acceleration equation: %.15e*a + %.15e = 0 with a being in (-inf, inf)", coeff0%coeff1);
            ba = 0;
        }
        return ba;
    }

private:
    size_t _ndof;
    int _envid;

    // Caching stuff
    std::vector<dReal> _cacheVect, _cacheSwitchpointsList;
    std::vector<dReal> _cacheX0Vect, _cacheX1Vect, _cacheV0Vect, _cacheV1Vect, _cacheAVect;
    Ramp _cacheRamp;
    std::vector<Ramp> _cacheRampsVect;
    std::vector<Ramp> _cacheRampsVect2; // for using in Compute1DTrajectoryFixedDuration
    ParabolicCurve _cacheCurve;
    std::vector<ParabolicCurve> _cacheCurvesVect;
};

} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE

#endif
