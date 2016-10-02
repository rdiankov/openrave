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
#include <openrave/openrave.h>
#include "paraboliccommon.h"

namespace OpenRAVE {

namespace RampOptimizerInternal {

#define DATA_OFFSET_X0 0
#define DATA_OFFSET_X1 1
#define DATA_OFFSET_V0 2
#define DATA_OFFSET_V1 3
#define DATA_OFFSET_A 4

#define IT_X0_BEGIN(data, ndof) ((data).begin() + DATA_OFFSET_X0*ndof)
#define IT_X0_END(data, ndof) ((data).begin() + DATA_OFFSET_X0*ndof + ndof)
#define IT_X1_BEGIN(data, ndof) ((data).begin() + DATA_OFFSET_X1*ndof)
#define IT_X1_END(data, ndof) ((data).begin() + DATA_OFFSET_X1*ndof + ndof)
#define IT_V0_BEGIN(data, ndof) ((data).begin() + DATA_OFFSET_V0*ndof)
#define IT_V0_END(data, ndof) ((data).begin() + DATA_OFFSET_V0*ndof + ndof)
#define IT_V1_BEGIN(data, ndof) ((data).begin() + DATA_OFFSET_V1*ndof)
#define IT_V1_END(data, ndof) ((data).begin() + DATA_OFFSET_V1*ndof + ndof)
#define IT_A_BEGIN(data, ndof) ((data).begin() + DATA_OFFSET_A*ndof)
#define IT_A_END(data, ndof) ((data).begin() + DATA_OFFSET_A*ndof + ndof)

class Ramp {
public:
    Ramp() {
    }
    /**
       \params v0_ initial velocity
       \params a_ acceleration
       \params duration_ duration of this ramp
       \params x0_ initial displacement. If not given, will set to zero.
     */
    Ramp(dReal v0_, dReal a_, dReal duration_, dReal x0_=0);
    ~Ramp() {
    }

    // Functions
    /// \brief Evaluate the position at time t, taking into account the initial displacement x0
    dReal EvalPos(dReal t) const;

    /// \brief Evaluate the velocity at time t
    dReal EvalVel(dReal t) const;

    /// \brief Evaluate the acceleration at time t
    dReal EvalAcc() const;

    /// \brief Get the minimum and maximum values of the ramp occuring in [0, duration]
    void GetPeaks(dReal& bmin, dReal& bmax) const;

    /// \brief Get the minimum and maximum values of the ramp occuring in [ta, tb]
    void GetPeaks(const dReal ta, dReal tb, dReal& bmin, dReal& bmax) const;

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

    /// \brief Append the given ParabolicCurve to this. This function may invalidate curve.
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

    void Initialize(Ramp& rampIn);

    void PrepareRampsVector(size_t nramps);

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

    /// \brief Return a constant reference to _ramps
    inline const std::vector<Ramp>& GetRamps() const
    {
        return _ramps;
    }

    /// \brief Return a constant reference to Ramp of the given index
    inline const Ramp& GetRamp(size_t index) const
    {
        return _ramps[index];
    }

    /// \brief Get the total duration of the curve
    inline const dReal GetDuration() const
    {
        return _duration;
    }

    /// \brief Get the initial velocity of the curve
    inline const dReal GetV0() const
    {
        return _ramps.at(0).v0;
    }

    /// \brief Get the final velocity of the curve
    inline const dReal GetV1() const
    {
        return _ramps.back().v1;
    }

    /// \brief Get the initial position of the curve
    inline const dReal GetX0() const
    {
        return _ramps.at(0).x0;
    }

    /// \brief Get the final position of the curve
    inline const dReal GetX1() const
    {
        return _ramps.back().x1;
    }

    /// \brief Get the total displacement done by the curve
    inline const dReal GetTotalDisplacement() const
    {
        return _d;
    }

private:
    dReal _d;        // total displacement done by this curve
    dReal _duration; // total duration of this curve
    std::vector<Ramp> _ramps; // vector of all Ramps constituting this curve
}; // end class ParabolicCurve

class RampND {
public:
    RampND() {
        constraintChecked = false;
    }
    RampND(size_t ndof);

    /**
       \brief Initialize RampND to hold the given values. There can be X different initializations
       as follows:

       1. All values (x0Vect, x1Vect, v0Vect, v1Vect, aVect, t) are given.

       2. Only some values are given: x0Vect, x1Vect, v0Vect, v1Vect, t are given. The acceleration
          will be calculated to minimize the sum of square of errors between computed x1, v1 and the
          given x1, v1.
     */
    RampND(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect, const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect, const std::vector<dReal>& aVect, const dReal t);
    ~RampND() {
    }

    /// \brief Evaluate the position at time t
    void EvalPos(dReal t, std::vector<dReal>::iterator it) const;

    /// \brief Evaluate the velocity at time t
    void EvalVel(dReal t, std::vector<dReal>::iterator it) const;

    /// \brief Evaluate the acceleration at time t
    void EvalAcc(std::vector<dReal>::iterator it) const;

    /// \brief Evaluate the position at time t
    void EvalPos(dReal t, std::vector<dReal>& xVect) const;

    /// \brief Evaluate the velocity at time t
    void EvalVel(dReal t, std::vector<dReal>& vVect) const;

    /// \brief Evaluate the acceleration at time t
    void EvalAcc(std::vector<dReal>& aVect) const;

    /// \brief Initialize rampnd for storing ndof segment.
    void Initialize(size_t ndof);

    /// \brief Initialize rampnd from boundary values.
    void Initialize(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect, const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect, const std::vector<dReal>& aVect, const dReal t);

    /// \brief Set the rampnd to have zero acceleration, start with xVect, and have the specified duration.
    void SetConstant(const std::vector<dReal>& xVect, const dReal t);

    /// \brief Set the initial position to xVect and update x1Vect accordingly.
    void SetInitialPosition(const std::vector<dReal>& xVect);

    /// \brief Cut the rampnd into two halves at time t. The left half is stored in the same
    /// rampnd. The right half is returned via remRampND.
    void Cut(dReal t, RampND& remRampND);

    /// \brief Cut the rampnd into two halves at time t and keep the right half.
    void TrimFront(dReal t);

    /// \brief Cut the rampnd into two halves at time t and keep the left half.
    void TrimBack(dReal t);

    inline size_t GetDOF() const
    {
        return _ndof;
    }

    /// Before calling Get functions, users need to make sure that _data has been
    /// initialized beforehand.
    inline const dReal GetX0At(int idof) const
    {
        return _data[DATA_OFFSET_X0*_ndof + idof];
    }

    inline const dReal GetX1At(int idof) const
    {
        return _data[DATA_OFFSET_X1*_ndof + idof];
    }

    inline const dReal GetV0At(int idof) const
    {
        return _data[DATA_OFFSET_V0*_ndof + idof];
    }

    inline const dReal GetV1At(int idof) const
    {
        return _data[DATA_OFFSET_V1*_ndof + idof];
    }

    inline const dReal GetAAt(int idof) const
    {
        return _data[DATA_OFFSET_A*_ndof + idof];
    }

    inline dReal& GetX0At(int idof)
    {
        return _data[DATA_OFFSET_X0*_ndof + idof];
    }

    inline dReal& GetX1At(int idof)
    {
        return _data[DATA_OFFSET_X1*_ndof + idof];
    }

    inline dReal& GetV0At(int idof)
    {
        return _data[DATA_OFFSET_V0*_ndof + idof];
    }

    inline dReal& GetV1At(int idof)
    {
        return _data[DATA_OFFSET_V1*_ndof + idof];
    }

    inline dReal& GetAAt(int idof)
    {
        return _data[DATA_OFFSET_A*_ndof + idof];
    }

    inline const dReal GetDuration() const
    {
        return _duration;
    }

    inline void GetX0Vect(std::vector<dReal>& xVect) const
    {
        return _GetData(xVect, DATA_OFFSET_X0*_ndof);
    }

    inline void GetX1Vect(std::vector<dReal>& xVect) const
    {
        return _GetData(xVect, DATA_OFFSET_X1*_ndof);
    }

    inline void GetV0Vect(std::vector<dReal>& vVect) const
    {
        return _GetData(vVect, DATA_OFFSET_V0*_ndof);
    }

    inline void GetV1Vect(std::vector<dReal>& vVect) const
    {
        return _GetData(vVect, DATA_OFFSET_V1*_ndof);
    }

    inline void GetAVect(std::vector<dReal>& aVect) const
    {
        return _GetData(aVect, DATA_OFFSET_A*_ndof);
    }

    inline void _GetData(std::vector<dReal>& res, int offset) const
    {
        res.resize(_ndof);
        std::copy(_data.begin() + offset, _data.begin() + offset + _ndof, res.begin());
        return;
    }

    /// Value setting
    inline void SetDuration(dReal t)
    {
        OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
        _duration = t;
        return;
    }

    // Set values by giving a vector
    inline void SetX0Vect(const std::vector<dReal>& xVect)
    {
        return _SetData(xVect, DATA_OFFSET_X0*_ndof);
    }

    inline void SetX1Vect(const std::vector<dReal>& xVect)
    {
        return _SetData(xVect, DATA_OFFSET_X1*_ndof);
    }

    inline void SetV0Vect(const std::vector<dReal>& vVect)
    {
        return _SetData(vVect, DATA_OFFSET_V0*_ndof);
    }

    inline void SetV1Vect(const std::vector<dReal>& vVect)
    {
        return _SetData(vVect, DATA_OFFSET_V1*_ndof);
    }

    inline void SetAVect(const std::vector<dReal>& aVect)
    {
        return _SetData(aVect, DATA_OFFSET_A*_ndof);
    }

    inline void _SetData(const std::vector<dReal>& valueVect, int offset)
    {
        OPENRAVE_ASSERT_OP(valueVect.size(), ==, _ndof);
        std::copy(valueVect.begin(), valueVect.end(), _data.begin() + offset);
        return;
    }

    // Get/Set value by giving a pointer to the first element in the vector. Users need to make sure
    // that the given iterator is pointing to a vector of dimension consistent with _ndof

    // These functions may not be that useful. Will see later if we should remove them.
    inline std::vector<dReal>::const_iterator GetX0Vect() const
    {
        return IT_X0_BEGIN(_data, _ndof);
    }

    inline std::vector<dReal>::const_iterator GetX1Vect() const
    {
        return IT_X1_BEGIN(_data, _ndof);
    }

    inline std::vector<dReal>::const_iterator GetV0Vect() const
    {
        return IT_V0_BEGIN(_data, _ndof);
    }

    inline std::vector<dReal>::const_iterator GetV1Vect() const
    {
        return IT_V1_BEGIN(_data, _ndof);
    }

    inline std::vector<dReal>::const_iterator GetAVect() const
    {
        return IT_A_BEGIN(_data, _ndof);
    }

    inline void SetX0Vect(std::vector<dReal>::const_iterator it)
    {
        return _SetData(it, 0);
    }

    inline void SetX1Vect(std::vector<dReal>::const_iterator it)
    {
        return _SetData(it, _ndof);
    }

    inline void SetV0Vect(std::vector<dReal>::const_iterator it)
    {
        return _SetData(it, 2*_ndof);
    }

    inline void SetV1Vect(std::vector<dReal>::const_iterator it)
    {
        return _SetData(it, 3*_ndof);
    }

    inline void SetAVect(std::vector<dReal>::const_iterator it)
    {
        return _SetData(it, 4*_ndof);
    }

    inline void _SetData(std::vector<dReal>::const_iterator it, int offset)
    {
        for (size_t idof = 0; idof < _ndof; ++idof) {
            *(_data.begin() + offset + idof) = *(it + idof);
        }
        return;
    }

    /// \brief Serialize this RampND. The format is _ndof<space>all entries in _data<space>_duration
    void Serialize(std::ostream& O) const;

    /**
       constraintChecked indicates if this RampND has been checked with all the constraints. (This
       parameter will be manipulated by the parabolicsmoother.)

       This is neccessary because checkings of the same straight line path with two different
       interpolation (i.e., linear and parabolic interpolation) can disagree due to different
       discretization resolution. The idea is to use constraintChecked as a flag to tell the
       parabolicsmoother to trust the feasibility guarantee from linearsmoother.
     */
    mutable bool constraintChecked;

private:
    size_t _ndof;
    dReal _duration;
    std::vector<dReal> _data; // a vector of size 5*_ndof containing the data of the following
                              // order: x0Vect, x1Vect, v0Vect, v1Vect, and aVect.
}; // end class RampND

class ParabolicPath {
public:
    ParabolicPath() {
        _duration = 0;
    }
    ~ParabolicPath() {
    }

    /// \brief Append the given rampnd to rampsnds and update related values
    void AppendRampND(RampND& rampndIn);

    /// \brief Evaluate the position at time t
    void EvalPos(dReal t, std::vector<dReal>& xVect) const;

    /// \brief Evaluate the velocity at time t
    void EvalVel(dReal t, std::vector<dReal>& vVect) const;

    /// \brief Evaluate the acceleration at time t
    void EvalAcc(dReal t, std::vector<dReal>& aVect) const;

    /// \brief Find the index of rampnd that t falls into and also compute the remainder
    void FindRampNDIndex(dReal t, int& index, dReal& remainder) const;

    /// \brief Initialize this parabolicpath with the given vector of rampnd
    void Initialize(const RampND& rampndIn);

    /// \brief Replace anything between the time instants t0 and t1 with the given sequence of
    /// rampnd.
    void ReplaceSegment(dReal t0, dReal t1, const std::vector<RampND>& rampndIn);

    /// \brief Reset this ParabolicPath. _switchpointsList will be reset to a vector of size 1,
    /// containing zero.
    inline void Reset() {
        _duration = 0;
        _rampnds.resize(0);
    }

    /// \brief Serialize the parabolicpath to stream for saving to file.
    void Serialize(std::ostream& O) const;

    /// \brief To be called after some changes have been made to _rampnds in order to update
    /// _duration accordingly.
    void _UpdateDuration();

    inline dReal GetDuration() const
    {
        return _duration;
    }

    inline const std::vector<RampND>& GetRampNDVect() const
    {
        return _rampnds;
    }

private:
    std::vector<RampND> _rampnds;
    dReal _duration;
}; // end class ParabolicPath

} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
#endif
