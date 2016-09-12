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
#ifndef RAMP_OPTIM_RAMP_H
#define RAMP_OPTIM_RAMP_H

#include <vector>
#include <cassert>
#include <openrave/openrave.h>
#include <boost/format.hpp>
#include "newparaboliccommon.h"

namespace OpenRAVE {

namespace RampOptimizerInternal {

class Ramp {
public:
    Ramp() {
    }
    Ramp(dReal v0_, dReal a_, dReal duration_, dReal x0_=0);
    ~Ramp() {
    }

    // Functions
    /// \brief Evaluate the position at time t, taking into account the initial displacement x0
    dReal EvalPos(dReal t) const;
    /// \brief Evaluate the velocity at time t
    dReal EvalVel(dReal t) const;
    /// \brief Evaluate the acceleration at time t
    dReal EvalAcc(dReal t) const;
    /// \brief Get the minimum and maximum values of the ramp occuring in [0, duration]
    void GetPeaks(dReal& bmin, dReal& bmax) const;
    /// \brief Get the minimum and maximum values of the ramp occuring in [ta, tb]
    void GetPeaks(dReal ta, dReal tb, dReal& bmin, dReal& bmax) const;
    /// \brief Assign new values to class members
    void Initialize(dReal v0_, dReal a_, dReal duration_, dReal x0_=0);
    /// \brief Set x0 to newx0 and updated related values accordingly
    void SetInitialValue(dReal newx0);
    /// \brief Set duration to newDuration and updated related values accordingly
    void UpdateDuration(dReal newDuration);

    /// \brief Copy all members from the given ramp
    void Copy(Ramp& outRamp);
    /// \brief Cut the Ramp into two halves at time t. The left half is stored in the same Ramp. The
    /// right half is returned via remRamp
    void Cut(dReal t, Ramp &remRamp);
    /// \brief Cut the Ramp into two halves at time t and keep the right half
    void TrimFront(dReal t);
    /// \brief Cut the Ramp into two halves at time t and keep the left half
    void TrimBack(dReal t);

    // Members
    dReal v0;       // initial velocity
    dReal a;        // acceleration
    dReal duration; // total duration
    dReal x0;       // initial displacement
    dReal x1;       // final displacement
    dReal v1;       // final velocity
    dReal d;        // total displacement 'done' by this Ramp. For example, EvalPos(duration) = x0 + d = x1
}; // end class Ramp

class ParabolicCurve {
public:
    ParabolicCurve() {
    }
    ParabolicCurve(std::vector<Ramp>& rampsIn);
    ~ParabolicCurve() {
    }

    /// \brief Append the given ParabolicCurve to this.
    void Append(ParabolicCurve& curve);
    /// \brief Evaluate the position at time t
    dReal EvalPos(dReal t) const;
    /// \brief Evaluate the velocity at time t
    dReal EvalVel(dReal t) const;
    /// \brief Evaluate the acceleration at time t
    dReal EvalAcc(dReal t) const;
    /// \brief Find the index of the ramp that t falls into and also compute the remainder.
    void FindRampIndex(dReal t, int& i, dReal& rem) const;
    /// \brief Get the minimum and maximum values of the curve occuring in [0, duration]
    void GetPeaks(dReal& bmin, dReal& bmax) const;
    /// \brief Get the minimum and maximum values of the curve occuring in [ta, tb]
    void GetPeaks(dReal ta, dReal tb, dReal& bmin, dReal& bmax) const;
    /// \brief Initialize this with a new set of Ramps. This function will invalidate rampsIn
    /// because _ramps will be swapped with rampsIn.
    void Initialize(std::vector<Ramp>& rampsIn);
    /// \brief Return true if no ramp is stored in this curve. Return true otherwise.
    bool IsEmpty() const
    {
        return _ramps.size() == 0;
    }
    /// \brief Resize all vectors to zero and reset all dReal values to zero.
    void Reset();
    /// \brief Set the curve to be a constant segment, i.e., having zero terminal velocities and
    /// zero accelration.
    void SetConstant(dReal x0, dReal t=0);
    /// \brief Set the initial value of the first ramp (in _ramps) to newx0 and compute/change all
    /// other related values accordingly.
    void SetInitialValue(dReal newx0);
    /// \brief Set the curve to contain one ramp connecting (x0, v0) and (x1, v1) in time t. Note
    /// that this function assumes consistency of inputs.
    void SetSegment(dReal x0, dReal x1, dReal v0, dReal v1, dReal t);
    /// \brief Set the curve to be one configuration (x0, v0).
    void SetZeroDuration(dReal x0, dReal v0);
    /// \brief Efficiently swap this and anotherCurve.
    void Swap(ParabolicCurve& anotherCurve);
    /// \brief Cut the ParabolicCurve into two halves at time t. The left half is stored in the same
    /// ParabolicCurve. The right half is returned via remCurve
    void Cut(dReal t, ParabolicCurve &remCurve);
    /// \brief Cut the ParabolicCurve into two halves at time t and keep the right half.
    void TrimFront(dReal t);
    /// \brief Cut the ParabolicCurve into two halves at time t and keep the left half.
    void TrimBack(dReal t);

    inline const std::vector<Ramp>& GetRamps() const
    {
        return _ramps;
    }

    inline const std::vector<dReal>& GetSwitchPointsList() const
    {
        return _switchpointsList;
    }
    /// \brief Get the total duration of the curve
    inline const dReal& GetDuration() const
    {
        return _duration;
    }
    /// \brief Get the initial velocity of the curve
    inline const dReal& GetV0() const
    {
        return _ramps[0].v0;
    }
    /// \brief Get the final velocity of the curve
    inline const dReal& GetV1() const
    {
        return _ramps[_ramps.size()].v1;
    }
    /// \brief Get the initial position of the curve
    inline const dReal& GetX0() const
    {
        return _ramps[0].x0;
    }
    /// \brief Get the final position of the curve
    inline const dReal& GetX1() const
    {
        return _ramps[_ramps.size()].x1;
    }
    /// \brief Get the total displacement done by the curve
    inline const dReal& GetTotalDisplacement() const
    {
        return _d;
    }

private:
    dReal _d;        // total displacement done by this curve
    dReal _duration; // total duration of this curve

    std::vector<Ramp> _ramps;             // vector of all Ramps constituting this curve
    std::vector<dReal> _switchpointsList; // vector of all time instants at which the acceleration
                                          // changes. This include t = 0 and t =
                                          // _duration. _switchpointsList.size() = _ramps.size() + 1
                                          // must always hold.
}; // end class ParabolicCurve

class ParabolicCurvesND {
public:
    ParabolicCurvesND() {
    }
    ParabolicCurvesND(std::vector<ParabolicCurve> curves);
    ParabolicCurvesND(int ndof);
    ~ParabolicCurvesND() {
    }

    /// \brief Append the given curvesnd to this
    void Append(ParabolicCurvesND curvesnd);
    /// \brief Evaluate the position at time t
    void EvalPos(dReal t, std::vector<dReal>& xVect) const;
    /// \brief Evaluate the velocity at time t
    void EvalVel(dReal t, std::vector<dReal>& vVect) const;
    /// \brief Evaluate the acceleration at time t
    void EvalAcc(dReal t, std::vector<dReal>& aVect) const;
    /// \brief Initialize the curvesnd with the new set of curves
    void Initialize(std::vector<ParabolicCurve> curves);
    /// \brief Return true if no curve is stored in this curvesnd. Return false otherwise.
    bool IsEmpty() const {
        return curves.size() == 0;
    }
    /// \brief Resize all vectors to zero.
    void Reset();
    /// \brief Set the initial value of the curvesnd to _x0Vect. Also recalculate related values accordingly.
    void SetInitialValues(const std::vector<dReal>& x0Vect);
    /// \brief Set the curvesnd to be a constant (stationary) curvesnd
    void SetConstant(const std::vector<dReal>& x0Vect, dReal t=0);
    /// \brief Set the curvesnd to be straight lines interpolating (x0Vect, v0Vect) and (x1Vect,
    /// v1Vect) in time t. All inputs are assumed to be consistent.
    void SetSegment(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect, const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect, dReal t);
    /// \brief Set the curvesnd to be just one configuration (x0Vect, v0Vect)
    void SetZeroDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& v0Vect);

    /// \brief Cut the ParabolicCurvesND into two halves at time t. The left half is stored in the
    /// same ParabolicCurvesND. The right half is returned via remCurvesND
    void Cut(dReal t, ParabolicCurvesND &remCurvesND);
    /// \brief Cut the ParabolicCurvesND into two halves at time t and keep the right half.
    void TrimFront(dReal t);
    /// \brief Cut the ParabolicCurvesND into two halves at time t and keep the left half.
    void TrimBack(dReal t);
    ///\brief Serialize the ParabolicCurvesND into string (for saving to file)
    void Serialize(std::ostream &O) const;

    inline const dReal& GetV0At(size_t idof) const
    {
        return data[idof];
    }
    
    inline const dReal& GetAAt(size_t idof) const
    {
        return data[_ndof + idof];
    }
    
    inline const dReal& GetX0At(size_t idof) const
    {
        return data[2*_ndof + idof];
    }

    inline const dReal& GetX1At(size_t idof) const
    {
        return data[3*_ndof + idof];
    }

    inline const dReal& GetV1At(size_t idof) const
    {
        return data[4*_ndof + idof];
    }

    inline const dReal& GetDAt(size_t idof) const
    {
        return data[5*_ndof + idof];
    }

    inline const dReal& GetDuration() const
    {
        return data[data.size() - 1];
    }

private:
    size_t _ndof;

    std::vector<dReal> _data; // data contains all boundary conditions of this curvesnd. The
                              // ordering is as follows. data = [v0Vect, aVect, x0Vect, x1Vect,
                              // v1Vect, dVect, t]
    std::vector<ParabolicCurve> _curves;
    std::vector<dReal> _switchpointsList;


}; // end class ParabolicCurvesND

} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
