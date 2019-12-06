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
#include "ramp.h"
#include <iostream>

namespace OpenRAVE {

namespace RampOptimizerInternal {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Ramp
Ramp::Ramp(dReal v0_, dReal a_, dReal duration_, dReal x0_)
{
    OPENRAVE_ASSERT_OP(duration_, >=, -g_fRampEpsilon);

    v0 = v0_;
    a = a_;
    duration = duration_;
    x0 = x0_;

    v1 = v0 + (a*duration);
    d = duration*(v0 + 0.5*a*duration);
    x1 = x0 + d;
}

dReal Ramp::EvalPos(dReal t) const
{
    if (t <= 0) {
        return x0;
    }
    else if (t >= duration) {
        return x1;
    }

    return x0 + t*(v0 + 0.5*a*t);
}

dReal Ramp::EvalVel(dReal t) const
{
    if (t <= 0) {
        return v0;
    }
    else if (t >= duration) {
        return v1;
    }

    return v0 + (a*t);
}

dReal Ramp::EvalAcc() const
{
    return a;
}

void Ramp::GetPeaks(dReal& bmin, dReal& bmax) const
{
    return GetPeaks(0, duration, bmin, bmax);
}

void Ramp::GetPeaks(dReal ta, dReal tb, dReal& bmin, dReal& bmax) const
{
    if( ta > tb ) {
        GetPeaks(tb, ta, bmin, bmax);
        return;
    }

    if( ta < 0 ) {
        ta = 0;
    }
    if( tb <= 0 ) {
        bmin = x0;
        bmax = x0;
        return;
    }

    if( tb > duration ) {
        tb = duration;
    }
    if( ta >= duration ) {
        bmin = x1;
        bmax = x1;
        return;
    }

    if( FuzzyZero(a, g_fRampEpsilon) ) {
        if( v0 > 0 ) {
            bmin = EvalPos(ta);
            bmax = EvalPos(tb);
        }
        else {
            bmin = EvalPos(tb);
            bmax = EvalPos(ta);
        }
        return;
    }

    dReal curMin, curMax;
    curMin = EvalPos(ta);
    curMax = EvalPos(tb);
    if( curMin > curMax ) {
        Swap(curMin, curMax);
    }

    dReal tDeflection = -v0/a; // the time when velocity crosses zero
    if( (tDeflection <= ta) || (tDeflection >= tb) ) {
        bmin = curMin;
        bmax = curMax;
        return;
    }

    dReal xDeflection = x0 + 0.5*v0*tDeflection;
    bmin = Min(curMin, xDeflection);
    bmax = Max(curMax, xDeflection);
    return;
}

void Ramp::Initialize(dReal v0_, dReal a_, dReal duration_, dReal x0_)
{
    OPENRAVE_ASSERT_OP(duration_, >=, -g_fRampEpsilon);

    v0 = v0_;
    a = a_;
    duration = duration_;
    x0 = x0_;

    v1 = v0 + (a*duration);
    d = duration*(v0 + 0.5*a*duration);
    x1 = x0 + d;
    return;
}

void Ramp::SetInitialValue(dReal newx0)
{
    x0 = newx0;
    x1 = x0 + d;
    return;
}

void Ramp::UpdateDuration(dReal newDuration)
{
    OPENRAVE_ASSERT_OP(newDuration, >=, -g_fRampEpsilon);
    if( newDuration <= 0 ) {
        duration = 0;
        x1 = x0;
        v1 = v0;
        d = 0;
        return;
    }

    duration = newDuration;
    v1 = v0 + (a*duration);
    d = duration*(v0 + 0.5*a*duration);
    x1 = x0 + d;
    return;
}

void Ramp::Copy(Ramp& outRamp)
{
    x0 = outRamp.x0;
    x1 = outRamp.x1;
    v0 = outRamp.v0;
    v1 = outRamp.v1;
    a = outRamp.a;
    duration = outRamp.duration;
    d = outRamp.d;
}

void Ramp::Cut(dReal t, Ramp& remRamp)
{
    if( t <= 0 ) {
        remRamp.Copy(*this);
        Initialize(v0, 0, 0, x0);
        return;
    }
    else if( t >= duration ) {
        remRamp.Initialize(v1, 0, 0, x1);
        return;
    }

    dReal remRampDuration = duration - t;
    UpdateDuration(t);
    remRamp.Initialize(v1, a, remRampDuration, x1);
    return;
}

void Ramp::TrimFront(dReal t)
{
    if( t <= 0 ) {
        return;
    }
    else if( t >= duration ) {
        Initialize(v1, 0, 0, x1);
        return;
    }

    dReal newx0 = EvalPos(t);
    dReal newv0 = EvalVel(t);
    Initialize(newv0, a, duration - t, newx0);
    return;
}

void Ramp::TrimBack(dReal t)
{
    if( t <= 0 ) {
        Initialize(v0, 0, 0, x0);
        return;
    }
    else if( t >= duration ) {
        return;
    }

    UpdateDuration(t);
    return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// ParabolicCurve
ParabolicCurve::ParabolicCurve(std::vector<Ramp>&rampsIn)
{
    OPENRAVE_ASSERT_OP(rampsIn.size(), >, 0);

    // This will invalidate rampsIn
    _ramps.swap(rampsIn);

    dReal d = 0;
    dReal duration = 0;

    int index = 1;
    for (std::vector<Ramp>::const_iterator itramp = _ramps.begin(); itramp != _ramps.end(); ++itramp, ++index) {
        d += itramp->d;
        duration += itramp->duration;
    }
    _d = d;
    _duration = duration;

    // Call SetInitialValue to enforce consistency of position throughout the curve (i.e., enforcing
    // ramps[i - 1].x1 = ramps[i].x0).
    SetInitialValue(_ramps[0].x0);
    return;
}

void ParabolicCurve::Append(ParabolicCurve& curve)
{
    if( IsEmpty() ) {
        // This will invalidate curve
        _ramps.swap(curve._ramps);
        _d = curve._d;
        _duration = curve._duration;
        return;
    }

    if( _ramps.capacity() < _ramps.size() + curve._ramps.size() ) {
        _ramps.reserve(_ramps.size() + curve._ramps.size());
    }
    curve.SetInitialValue(_d); // This will invalidate the curve
    _d = curve._d;
    _duration += curve._duration;
    _ramps.insert(_ramps.end(), curve._ramps.begin(), curve._ramps.end());
}

dReal ParabolicCurve::EvalPos(dReal t) const
{
    if( t <= 0 ) {
        return _ramps.front().x0;
    }
    else if( t >= _duration ) {
        return _ramps.back().x1;
    }

    int index;
    dReal remainder;
    FindRampIndex(t, index, remainder);
    return _ramps[index].EvalPos(remainder);
}

dReal ParabolicCurve::EvalVel(dReal t) const
{
    if( t <= 0 ) {
        return _ramps.front().v0;
    }
    else if( t >= _duration ) {
        return _ramps.back().v1;
    }

    int index;
    dReal remainder;
    FindRampIndex(t, index, remainder);
    return _ramps[index].EvalVel(remainder);
}

dReal ParabolicCurve::EvalAcc(dReal t) const
{
    if( t <= 0 ) {
        return _ramps.front().a;
    }
    else if( t >= _duration ) {
        return _ramps.back().a;
    }

    // Note that when t is at a switch point, the returned value is depending on the behavior of
    // FindRampIndex.
    int index;
    dReal remainder;
    FindRampIndex(t, index, remainder);
    return _ramps[index].a;
}

void ParabolicCurve::FindRampIndex(dReal t, int& index, dReal& remainder) const
{
    if( t <= 0 ) {
        index = 0;
        remainder = 0;
        return;
    }
    else if( t >= _duration ) {
        index = ((int) _ramps.size() - 1);
        remainder = _ramps.back().duration;
        return;
    }
    else {
        // Convention: if t lies exactly at a switch point between ramps[i] and ramps[i + 1], we
        // return index = i + 1 and remainder = 0.
        int index_ = 0;
        dReal curTime = 0;
        std::vector<Ramp>::const_iterator itramp = _ramps.begin();
        while ((itramp - 1) != _ramps.end() && t >= curTime) {
            curTime += itramp->duration;
            itramp++;
            index_++;
        }
        index = index_ - 1;
        remainder = t - (curTime - (itramp - 1)->duration);
        return;
    }
}

void ParabolicCurve::GetPeaks(dReal& bmin, dReal& bmax) const
{
    return GetPeaks(0, _duration, bmin, bmax);
}

void ParabolicCurve::GetPeaks(dReal ta, dReal tb, dReal& bmin, dReal& bmax) const
{
    dReal curMin = g_fRampInf, curMax = -g_fRampInf;
    dReal tempMin, tempMax;
    for (std::vector<Ramp>::const_iterator itramp = _ramps.begin(); itramp != _ramps.end(); ++itramp) {
        itramp->GetPeaks(ta, tb, tempMin, tempMax);
        if( tempMin < curMin ) {
            curMin = tempMin;
        }
        if( tempMax > curMax ) {
            curMax = tempMax;
        }
    }
    bmin = curMin;
    bmax = curMax;
    return;
}

void ParabolicCurve::Initialize(std::vector<Ramp>& rampsIn)
{
    // The same as in the constructor
    OPENRAVE_ASSERT_OP(rampsIn.size(), >, 0);

    // This will invalidate rampsIn
    _ramps.swap(rampsIn);

    dReal d = 0;
    dReal duration = 0;

    int index = 1;
    for (std::vector<Ramp>::const_iterator itramp = _ramps.begin(); itramp != _ramps.end(); ++itramp, ++index) {
        d += itramp->d;
        duration += itramp->duration;
    }
    _d = d;
    _duration = duration;

    // Call SetInitialValue to enforce consistency of position throughout the curve (i.e., enforcing
    // ramps[i - 1].x1 = ramps[i].x0).
    SetInitialValue(_ramps[0].x0);
    return;
}

void ParabolicCurve::Initialize(Ramp& rampIn)
{
    if( _ramps.size() != 1 ) {
        _ramps.resize(1);
    }
    _ramps[0] = rampIn;
    _d = rampIn.d;
    _duration = rampIn.duration;
}

void ParabolicCurve::Reset()
{
    _ramps.clear();
    _duration = 0;
    _d = 0;
    return;
}

void ParabolicCurve::SetConstant(dReal x0, dReal t)
{
    if( t < 0 ) {
        t = 0;
    }

    if( _ramps.size() != 1) {
        _ramps.resize(1);
    }
    _ramps[0].Initialize(0, 0, t, x0);
    _d = _ramps[0].d;
    _duration = t;
    return;
}

void ParabolicCurve::SetInitialValue(dReal newx0)
{
    for (std::vector<Ramp>::iterator itramp = _ramps.begin(); itramp != _ramps.end(); ++itramp) {
        itramp->SetInitialValue(newx0);
        newx0 += itramp->d;
    }
    return;
}

void ParabolicCurve::SetSegment(dReal x0, dReal x1, dReal v0, dReal v1, dReal t)
{
    if( t < 0 ) {
        t = 0;
    }

    /*
       There are two points to be noted.

       1. Normally, SetSegment will be called inside some check functions in parabolicsmoother after
         the segment has been checked. Therefore, we store the given boundary conditions as is.

       2. From 1. we therefore want to compute an acceleration which is the most consistent with all
         the given values. Assuming that x0, v0, and t are always correct, we want to compute a such
         that it minimizes differences between computed x1, v1, and actual x1, v1. That is

                 a* = argmin_a C(a) = (v0 + at - v1)^2 + (x0 + v0*t + 0.5*a*t^2 - x1)^2.

         >>>> import sympy as sp
         >>>> x0, x1, v0, v1, a, t = sp.symbols(['x0', 'x1', 'v0', 'v1', 'a', 't'])
         >>>> C = (v0 + a*t - v1)**2 + (x0 + v0*t + 0.5*a*t*t - x1)**2

         We can see that C(a) is a *convex* parabola, i.e., the coefficient of a^2 is strictly
         positive (given that t > 0). Therefore, it always has a minimum and the minimum is at

                 a* = -(v0*t^2 + t*(x0 - x1) + 2*(v0 - v1))/(0.5*t^3 + 2*t).

         >>>> sp.solve(sp.diff(C, a), a)
     */
    dReal a;
    if( FuzzyZero(t, g_fRampEpsilon) ) {
        a = 0;
    }
    else {
        dReal tSqr = t*t;
        a = -(v0*tSqr + t*(x0 - x1) + 2*(v0 - v1))/(t*(0.5*tSqr + 2));
    }

    if( _ramps.size() != 1) {
        _ramps.resize(1);
    }
    // Directly assign values to avoid recalculation.
    const std::vector<Ramp>::iterator itramp = _ramps.begin();
    itramp->x0 = x0;
    itramp->x1 = x1;
    itramp->v0 = v0;
    itramp->v1 = v1;
    itramp->duration = t;
    itramp->d = x1 - x0;
    itramp->a = a;

    _d = itramp->d;
    _duration = t;
    return;
}

void ParabolicCurve::SetZeroDuration(dReal x0, dReal v0)
{
    if( _ramps.size() != 1 ) {
        _ramps.resize(1);
    }
    _ramps[0].Initialize(v0, 0, 0, x0);

    _d = 0;
    _duration = 0;
    return;
}

void ParabolicCurve::Swap(ParabolicCurve& anotherCurve)
{
    _ramps.swap(anotherCurve._ramps);
    dReal temp;
    temp = _d;
    _d = anotherCurve._d;
    anotherCurve._d = temp;

    temp = _duration;
    _duration = anotherCurve._duration;
    anotherCurve._duration = temp;
}

void ParabolicCurve::Cut(dReal t, ParabolicCurve& remCurve)
{
    if( t <= 0 ) {
        Swap(remCurve);
        SetZeroDuration(remCurve.GetX0(), remCurve.GetV0());
        return;
    }
    else if( t >= _duration ) {
        remCurve.SetZeroDuration(GetX1(), GetV1());
        return;
    }

    int index;
    dReal remainder;
    FindRampIndex(t, index, remainder);

    if( remainder == 0 ) {
        // t is a switch point. No trimming required. Note also that index will always be greater
        // than 0.
        remCurve._ramps.resize(_ramps.size() - index);
        std::copy(_ramps.begin() + index, _ramps.end(), remCurve._ramps.begin());
        remCurve._d = remCurve.GetX1() - remCurve.GetX0();
        remCurve._duration = _duration - t;

        _ramps.resize(index);
        _d = GetX1() - GetX0();
        _duration = t;
        return;
    }
    else {
        // Note that index will always be strictly less than _ramps.size().
        remCurve._ramps.resize(_ramps.size() - index);
        std::copy(_ramps.begin() + index, _ramps.end(), remCurve._ramps.begin());
        remCurve._ramps.front().TrimFront(remainder);
        remCurve._d = remCurve.GetX1() - remCurve.GetX0();
        remCurve._duration = _duration - t;

        _ramps.resize(index + 1);
        _ramps.back().TrimBack(remainder);
        _d = GetX1() - GetX0();
        _duration = t;
    }
}

void ParabolicCurve::TrimFront(dReal t)
{
    if( t <= 0 ) {
        return;
    }
    else if( t >= _duration ) {
        SetZeroDuration(GetX1(), GetV1());
        return;
    }

    int index;
    dReal remainder;
    FindRampIndex(t, index, remainder);

    int rampssize = _ramps.size();
    if( index > 0 ) {
        for (int iramp = index; iramp < rampssize; ++iramp) {
            _ramps[iramp - index] = _ramps[iramp];
        }
        _ramps.resize(rampssize - index);
    }
    _ramps[0].TrimFront(remainder);
    _duration -= remainder;
    SetInitialValue(_ramps[0].x0);
    return;
}

void ParabolicCurve::TrimBack(dReal t)
{
    if( t <= 0 ) {
        SetZeroDuration(GetX0(), GetV0());
        return;
    }
    else if( t >= _duration ) {
        return;
    }

    int index;
    dReal remainder;
    FindRampIndex(t, index, remainder);

    if( remainder == 0 ) {
        // Here index is strictly positive since the case when index == 0 and remainder == 0 would
        // have been caught from if( t <= 0 ) above.
        _ramps.resize(index);
    }
    else {
        if( (int)_ramps.size() > index + 1 ) {
            _ramps.resize(index + 1);
        }
        _ramps.back().TrimBack(remainder);
    }
    _d = GetX1() - GetX0();
    _duration = t;
    return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// RampND
RampND::RampND(size_t ndof)
{
    OPENRAVE_ASSERT_OP(ndof, >, 0);
    _ndof = ndof;
    _data.resize(5*_ndof);
    constraintChecked = false;
}

RampND::RampND(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect, const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect, const std::vector<dReal>& aVect, dReal t)
{
    if( t < 0 ) {
        t = 0;
    }

    _ndof = x0Vect.size();
    OPENRAVE_ASSERT_OP(x1Vect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(v0Vect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(v1Vect.size(), ==, _ndof);
    // Acceleration and displacement vectors are optional
    if( aVect.size() > 0 ) {
        OPENRAVE_ASSERT_OP(aVect.size(), ==, _ndof);
    }
    _data.resize(5*_ndof);

    std::copy(x0Vect.begin(), x0Vect.end(), IT_X0_BEGIN(_data, _ndof));
    std::copy(x1Vect.begin(), x1Vect.end(), IT_X1_BEGIN(_data, _ndof));
    std::copy(v0Vect.begin(), v0Vect.end(), IT_V0_BEGIN(_data, _ndof));
    std::copy(v1Vect.begin(), v1Vect.end(), IT_V1_BEGIN(_data, _ndof));

    if( aVect.size() == 0 ) {
        if( t == 0 ) {
            std::fill(IT_A_BEGIN(_data, _ndof), IT_A_END(_data, _ndof), 0);
        }
        else {
            // Calculating accelerations using the same procedure as in ParabolicCurve::SetSegment.
            dReal tSqr = t*t;
            dReal divMult = 1/(t*(0.5*tSqr + 2));
            for (size_t idof = 0; idof < _ndof; ++idof) {
                _data[DATA_OFFSET_A*_ndof + idof] = -(v0Vect[idof]*tSqr + t*(x0Vect[idof] - x1Vect[idof]) + 2*(v0Vect[idof] - v1Vect[idof]))*divMult;
            }
        }
    }
    else {
        std::copy(aVect.begin(), aVect.end(), IT_A_BEGIN(_data, _ndof));
    }

    _duration = t;

    constraintChecked = false;
}

void RampND::EvalPos(dReal t, std::vector<dReal>::iterator it) const
{
    if( t <= 0 ) {
        std::copy(IT_X0_BEGIN(_data, _ndof), IT_X0_END(_data, _ndof), it);
        return;
    }
    else if( t >= GetDuration() ) {
        std::copy(IT_X1_BEGIN(_data, _ndof), IT_X1_END(_data, _ndof), it);
        return;
    }

    for (size_t idof = 0; idof < _ndof; ++idof) {
        *(it + idof) = GetX0At(idof) + t*(GetV0At(idof) + 0.5*t*GetAAt(idof));
    }
    return;
}

void RampND::EvalVel(dReal t, std::vector<dReal>::iterator it) const
{
    if( t <= 0 ) {
        std::copy(IT_V0_BEGIN(_data, _ndof), IT_V0_END(_data, _ndof), it);
        return;
    }
    else if( t >= GetDuration() ) {
        std::copy(IT_V1_BEGIN(_data, _ndof), IT_V1_END(_data, _ndof), it);
        return;
    }

    for (size_t idof = 0; idof < _ndof; ++idof) {
        *(it + idof) = GetV0At(idof) + t*GetAAt(idof);
    }
    return;
}

void RampND::EvalAcc(std::vector<dReal>::iterator it) const
{
    std::copy(IT_A_BEGIN(_data, _ndof), IT_A_END(_data, _ndof), it);
    return;
}

void RampND::EvalPos(dReal t, std::vector<dReal>& xVect) const
{
    if( t <= 0 ) {
        GetX0Vect(xVect);
        return;
    }
    else if( t >= GetDuration() ) {
        GetX1Vect(xVect);
        return;
    }

    xVect.resize(_ndof);
    for (size_t idof = 0; idof < _ndof; ++idof) {
        xVect[idof] = GetX0At(idof) + t*(GetV0At(idof) + 0.5*t*GetAAt(idof));
    }
    return;
}

void RampND::EvalVel(dReal t, std::vector<dReal>& vVect) const
{
    if( t <= 0 ) {
        GetV0Vect(vVect);
        return;
    }
    else if( t >= GetDuration() ) {
        GetV1Vect(vVect);
        return;
    }

    vVect.resize(_ndof);
    for (size_t idof = 0; idof < _ndof; ++idof) {
        vVect[idof] = GetV0At(idof) + t*GetAAt(idof);
    }
    return;
}

void RampND::EvalAcc(std::vector<dReal>& aVect) const
{
    GetAVect(aVect);
    return;
}

void RampND::Initialize(size_t ndof)
{
    constraintChecked = false;
    _ndof = ndof;
    if( _data.size() != 5*_ndof ) {
        _data.resize(5*_ndof);
    }
    std::fill(_data.begin(), _data.end(), 0);
}

void RampND::Initialize(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect, const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect, const std::vector<dReal>& aVect, dReal t)
{
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    if( t < 0 ) {
        t = 0;
    }

    _ndof = x0Vect.size();
    OPENRAVE_ASSERT_OP(x1Vect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(v0Vect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(v1Vect.size(), ==, _ndof);
    // Acceleration and displacement vectors are optional
    if( aVect.size() > 0 ) {
        OPENRAVE_ASSERT_OP(aVect.size(), ==, _ndof);
    }
    if( _data.size() != 5*_ndof ) {
        _data.resize(5*_ndof);
    }

    std::copy(x0Vect.begin(), x0Vect.end(), IT_X0_BEGIN(_data, _ndof));
    std::copy(x1Vect.begin(), x1Vect.end(), IT_X1_BEGIN(_data, _ndof));
    std::copy(v0Vect.begin(), v0Vect.end(), IT_V0_BEGIN(_data, _ndof));
    std::copy(v1Vect.begin(), v1Vect.end(), IT_V1_BEGIN(_data, _ndof));

    if( aVect.size() == 0 ) {
        if( t == 0 ) {
            std::fill(IT_A_BEGIN(_data, _ndof), IT_A_END(_data, _ndof), 0);
        }
        else {
            // Calculating accelerations using the same procedure as in ParabolicCurve::SetSegment.
            dReal tSqr = t*t;
            dReal divMult = 1/(t*(0.5*tSqr + 2));
            for (size_t idof = 0; idof < _ndof; ++idof) {
                _data[DATA_OFFSET_A*_ndof + idof] = -(v0Vect[idof]*tSqr + t*(x0Vect[idof] - x1Vect[idof]) + 2*(v0Vect[idof] - v1Vect[idof]))*divMult;
            }
        }
    }
    else {
        std::copy(aVect.begin(), aVect.end(), IT_A_BEGIN(_data, _ndof));
    }

    _duration = t;

    constraintChecked = false;
}

void RampND::SetConstant(const std::vector<dReal>& xVect, const dReal t)
{
    OPENRAVE_ASSERT_OP(xVect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    std::copy(xVect.begin(), xVect.end(), IT_X0_BEGIN(_data, _ndof)); // x0Vect
    std::copy(xVect.begin(), xVect.end(), IT_X1_BEGIN(_data, _ndof)); // x1Vect
    std::fill(IT_V0_BEGIN(_data, _ndof), _data.end(),  0); // v0Vect, v1Vect, aVect
    _duration = t; // duration
    return;
}

void RampND::SetInitialPosition(const std::vector<dReal>& xVect)
{
    OPENRAVE_ASSERT_OP(xVect.size(), ==, _ndof);
    dReal dOriginal;
    for (size_t idof = 0; idof < _ndof; ++idof) {
        dOriginal = GetX1At(idof) - GetX0At(idof);
        GetX0At(idof) = xVect[idof];
        GetX1At(idof) = xVect[idof] + dOriginal;
    }
    return;
}

void RampND::Cut(dReal t, RampND& remRampND)
{
    if( remRampND._ndof != _ndof ) {
        remRampND.Initialize(_ndof);
    }

    remRampND.constraintChecked = constraintChecked;

    if( t <= 0 ) {
        // Copy _data to remRampND._data
        std::copy(_data.begin(), _data.end(), remRampND._data.begin());
        remRampND._duration = _duration;

        // Update x1 of this
        std::copy(IT_X0_BEGIN(_data, _ndof), IT_X0_END(_data, _ndof), IT_X1_BEGIN(_data, _ndof));
        // Update v1 of this
        std::copy(IT_V0_BEGIN(_data, _ndof), IT_V0_END(_data, _ndof), IT_V1_BEGIN(_data, _ndof));

        _duration = 0;
        return;
    }
    else if( t >= _duration ) {
        // Update x0 of remRampND
        std::copy(IT_X1_BEGIN(_data, _ndof), IT_X1_END(_data, _ndof), IT_X0_BEGIN(remRampND._data, _ndof));
        // Update x1 of remRampND
        std::copy(IT_X1_BEGIN(_data, _ndof), IT_X1_END(_data, _ndof), IT_X1_BEGIN(remRampND._data, _ndof));
        // Update v0 of remRampND
        std::copy(IT_V1_BEGIN(_data, _ndof), IT_V1_END(_data, _ndof), IT_V0_BEGIN(remRampND._data, _ndof));
        // Update v1 of remRampND
        std::copy(IT_V1_BEGIN(_data, _ndof), IT_V1_END(_data, _ndof), IT_V1_BEGIN(remRampND._data, _ndof));
        // Update a of remRampND
        std::copy(IT_A_BEGIN(_data, _ndof), IT_A_END(_data, _ndof), IT_A_BEGIN(remRampND._data, _ndof));

        remRampND._duration = 0;
        return;
    }
    else {
        // Update x1 of remRampND
        std::copy(IT_X1_BEGIN(_data, _ndof), IT_X1_END(_data, _ndof), IT_X1_BEGIN(remRampND._data, _ndof));
        // Update v1 of remRampND
        std::copy(IT_V1_BEGIN(_data, _ndof), IT_V1_END(_data, _ndof), IT_V1_BEGIN(remRampND._data, _ndof));
        // Update a of remRampND
        std::copy(IT_A_BEGIN(_data, _ndof), IT_A_END(_data, _ndof), IT_A_BEGIN(remRampND._data, _ndof));

        // Update x1 of this
        EvalPos(t, IT_X1_BEGIN(_data, _ndof));
        // Update v1 of this. Note that the update of x1 does not affect this velocity calculation.
        EvalVel(t, IT_V1_BEGIN(_data, _ndof));

        // Update x0 of remRampND
        std::copy(IT_X1_BEGIN(_data, _ndof), IT_X1_END(_data, _ndof), IT_X0_BEGIN(remRampND._data, _ndof));
        // Update v0 of remRampND
        std::copy(IT_V1_BEGIN(_data, _ndof), IT_V1_END(_data, _ndof), IT_V0_BEGIN(remRampND._data, _ndof));

        // Update duration
        remRampND._duration = _duration - t;
        _duration = t;
        return;
    }
}

void RampND::TrimFront(dReal t)
{
    if( t <= 0 ) {
        return;
    }
    else if( t >= _duration ) {
        std::copy(IT_X1_BEGIN(_data, _ndof), IT_X1_END(_data, _ndof), IT_X0_BEGIN(_data, _ndof)); // replace x0 by x1
        std::copy(IT_V1_BEGIN(_data, _ndof), IT_V1_END(_data, _ndof), IT_V0_BEGIN(_data, _ndof)); // replace v0 by v1
        _duration = 0; // duration
        return;
    }
    else {
        EvalPos(t, IT_X0_BEGIN(_data, _ndof)); // update x0
        EvalVel(t, IT_V0_BEGIN(_data, _ndof)); // update v0. Note that the update of x0 does not affect velocity calculation
        _duration -= t;
        return;
    }
}

void RampND::TrimBack(dReal t)
{
    if( t <= 0 ) {
        std::copy(IT_X0_BEGIN(_data, _ndof), IT_X0_END(_data, _ndof), IT_X1_BEGIN(_data, _ndof)); // replace x1 by x0
        std::copy(IT_V0_BEGIN(_data, _ndof), IT_V0_END(_data, _ndof), IT_V1_BEGIN(_data, _ndof)); // replace v1 by v0
        _duration = 0; // duration
        return;
    }
    else if( t >= _duration ) {
        return;
    }
    else {
        EvalPos(t, IT_X1_BEGIN(_data, _ndof)); // update x1
        EvalVel(t, IT_V1_BEGIN(_data, _ndof)); // update v1. Note that the update of x1 does not affect velocity calculation
        _duration = t;
        return;
    }
}

void RampND::Serialize(std::ostream& O) const
{
    O << _ndof;
    for (size_t i = 0; i < _data.size(); ++i) {
        O << " " << _data[i];
    }
    O << " " << _duration << "\n";
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// ParabolicPath
void ParabolicPath::AppendRampND(RampND& rampndIn)
{
    _rampnds.resize(_rampnds.size() + 1);
    _rampnds.back() = rampndIn;
    _duration += rampndIn.GetDuration();
}

void ParabolicPath::EvalPos(dReal t, std::vector<dReal>& xVect) const
{
    int index;
    dReal remainder;
    FindRampNDIndex(t, index, remainder);
    _rampnds[index].EvalPos(remainder, xVect.begin());
}

void ParabolicPath::EvalVel(dReal t, std::vector<dReal>& vVect) const
{
    int index;
    dReal remainder;
    FindRampNDIndex(t, index, remainder);
    _rampnds[index].EvalVel(remainder, vVect.begin());
}

void ParabolicPath::EvalAcc(dReal t, std::vector<dReal>& aVect) const
{
    int index;
    dReal remainder;
    FindRampNDIndex(t, index, remainder);
    _rampnds[index].EvalAcc(aVect.begin());
}

void ParabolicPath::FindRampNDIndex(dReal t, int& index, dReal& remainder) const
{
    if( t <= 0 ) {
        index = 0;
        remainder = 0;
        return;
    }
    else if( t >= _duration ) {
        index = ((int)_rampnds.size() - 1);
        remainder = _rampnds.back().GetDuration();
        return;
    }
    else {
        // Convention: if t lies exactly at a switch point between rampsnd[i] and rampsnd[i + 1], we
        // return index = i + 1 and remainder = 0.
        int index_ = 0;
        dReal curTime = 0;
        std::vector<RampND>::const_iterator itrampnd = _rampnds.begin();
        while ((itrampnd - 1) != _rampnds.end() && t >= curTime) {
            curTime += itrampnd->GetDuration();
            itrampnd++;
            index_++;
        }
        index = index_ - 1;
        remainder = t - (curTime - (itrampnd - 1)->GetDuration());
        return;
    }
}

void ParabolicPath::Initialize(const RampND& rampndIn)
{
    _rampnds.resize(1);
    _rampnds[0] = rampndIn;
    _duration = rampndIn.GetDuration();
}

void ParabolicPath::ReplaceSegment(dReal t0, dReal t1, const std::vector<RampND>& rampndVect)
{
    OPENRAVE_ASSERT_OP(t0, <, t1);
    OPENRAVE_ASSERT_OP(rampndVect.size(), >, 0);
    if( t0 <= 0 && t1 >= _duration ) {
        dReal duration = 0;
        _rampnds = rampndVect;
        for (size_t irampnd = 0; irampnd < _rampnds.size(); ++irampnd) {
            duration += _rampnds[irampnd].GetDuration();
        }
        _duration = duration;
        return;
    }

    int index0, index1;
    dReal rem0, rem1;
    FindRampNDIndex(t0, index0, rem0);
    FindRampNDIndex(t1, index1, rem1);

    size_t prevSize = _rampnds.size();
    size_t leftPartLength = rem0 <= 0 ? index0 : (index0 + 1);
    size_t rightPartLength = t1 >= _duration ? 0 : (prevSize - index1);
    size_t newSize = leftPartLength + rampndVect.size() + rightPartLength;

    int newindex1 = index1; // index of the rampnd which contains t1 after the resizing
                            // operation. _rampnds.begin() + newindex1 will serve as the upper limit
                            // when replacing the segment using std::copy.

    // Make sure the right RampNDs are moved correctly
    if( prevSize < newSize ) {
        // The new size is greater than the original size. Resize the container first and start
        // moving from right to left.
        _rampnds.resize(newSize);
        for (size_t irampnd = 0; irampnd < rightPartLength; ++irampnd) {
            _rampnds[newSize - 1 - irampnd] = _rampnds[prevSize - 1 - irampnd];
        }
        newindex1 = newSize - rightPartLength;
    }
    else if( prevSize > newSize ) {
        // The new size is less than the current size. We need to move the RampNDs (from left to
        // right) first before resizing the container.
        for (size_t irampnd = 0; irampnd < rightPartLength; ++irampnd) {
            _rampnds[newSize - rightPartLength + irampnd] = _rampnds[prevSize - rightPartLength + irampnd];
        }
        _rampnds.resize(newSize);
        newindex1 = newSize - rightPartLength;
    }
    else {
        // All the right RampNDs stay at the same position
    }

    int newindex0 = index0 + 1; // _rampnds.begin() + newindex0 will be the start iterator when
                                // inserting the new segment.
    if( rem0 <= 0 ) {
        // In this case, we do not need to trim.
        newindex0 = index0;
    }
    else {
        _rampnds[index0].TrimBack(rem0);
    }

    if( t1 >= _duration ) {
        // In this case, we do not need to trim.
        newindex1 += 1;
    }
    else {
        _rampnds[newindex1].TrimFront(rem1);
    }
    OPENRAVE_ASSERT_OP(newindex0 + (int)rampndVect.size(), ==, newindex1);
    std::copy(rampndVect.begin(), rampndVect.end(), _rampnds.begin() + newindex0);

    _UpdateDuration();
}

void ParabolicPath::Serialize(std::ostream& O) const
{
    O << std::setprecision(g_nPrec);
    for (std::vector<RampND>::const_iterator itrampnd = _rampnds.begin(); itrampnd != _rampnds.end(); ++itrampnd) {
        itrampnd->Serialize(O);
    }
    return;
}

void ParabolicPath::_UpdateDuration()
{
    dReal duration = 0;
    for (size_t irampnd = 0; irampnd < _rampnds.size(); ++irampnd) {
        duration += _rampnds[irampnd].GetDuration();
    }
    _duration = duration;
}

} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
