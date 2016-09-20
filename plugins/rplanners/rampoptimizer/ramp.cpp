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
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, duration + g_fRampEpsilon);

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
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, duration + g_fRampEpsilon);

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

    dReal xDeflection = EvalPos(tDeflection);
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
    if( newDuration < 0 ) {
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
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, duration + g_fRampEpsilon);

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
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, duration + g_fRampEpsilon);

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
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, duration + g_fRampEpsilon);

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
    _switchpointsList.resize(0);
    if( _switchpointsList.size() < _ramps.size() + 1 ) {
        _switchpointsList.reserve(_ramps.size() + 1);
    }

    _d = 0;
    _duration = 0;
    _switchpointsList.push_back(_duration);

    for (std::vector<Ramp>::const_iterator itramp = _ramps.begin(); itramp != _ramps.end(); ++itramp) {
        _d += itramp->d;
        _duration += itramp->duration;
        _switchpointsList.push_back(_duration);
    }

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
        _switchpointsList.swap(curve._switchpointsList);
        _d = curve._d;
        _duration = curve._duration;
        return;
    }

    if( _ramps.capacity() < _ramps.size() + curve._ramps.size() ) {
        _ramps.reserve(_ramps.size() + curve._ramps.size());
    }
    if( _switchpointsList.capacity() < _switchpointsList.size() + curve._switchpointsList.size() - 1 ) {
        _switchpointsList.reserve(_switchpointsList.size() + curve._switchpointsList.size() - 1);
    }

    dReal prevDuration = _duration;
    curve.SetInitialValue(GetX1());
    for (size_t i = 0; i < curve._ramps.size(); ++i) {
        _ramps.push_back(curve._ramps[i]);
        // The following is always valid since _switchpointsList.size() = _ramps.size() + 1. Instead
        // of calculating new switch points by accumulating ramp durations, we add _duration to all
        // switch points in curve._switchpointsList in order to reduce numerical error.
        _switchpointsList.push_back(prevDuration + curve._switchpointsList[i + 1]);
    }
    _d += curve._d;
    _duration = _switchpointsList.back();
}

dReal ParabolicCurve::EvalPos(dReal t) const
{
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, _duration + g_fRampEpsilon);

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
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, _duration + g_fRampEpsilon);

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
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, _duration + g_fRampEpsilon);

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
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, _duration + g_fRampEpsilon);

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
        std::vector<dReal>::const_iterator it = _switchpointsList.begin();
        while( it != _switchpointsList.end() && t >= *it) {
            index_++;
            it++;
        }
        OPENRAVE_ASSERT_OP(index_, <, (int)_switchpointsList.size());
        index = index_ - 1;
        remainder = t - *(it - 1);
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
    _switchpointsList.resize(0);
    if( _switchpointsList.size() < _ramps.size() + 1 ) {
        _switchpointsList.reserve(_ramps.size() + 1);
    }

    _d = 0;
    _duration = 0;
    _switchpointsList.push_back(_duration);

    for (std::vector<Ramp>::const_iterator itramp = _ramps.begin(); itramp != _ramps.end(); ++itramp) {
        _d += itramp->d;
        _duration += itramp->duration;
        _switchpointsList.push_back(_duration);
    }

    // Call SetInitialValue to enforce consistency of position throughout the curve (i.e., enforcing
    // ramps[i - 1].x1 = ramps[i].x0).
    SetInitialValue(_ramps[0].x0);
    return;
}

void ParabolicCurve::Initialize(Ramp& rampIn)
{
    _ramps.resize(1);
    _ramps[0] = rampIn;
    _switchpointsList.resize(2);
    _switchpointsList[0] = 0;
    _switchpointsList[1] = rampIn.duration;
    _d = rampIn.d;
    _duration = rampIn.duration;
}

void ParabolicCurve::Reset()
{
    _ramps.clear();
    _switchpointsList.clear();
    _duration = 0;
    _d = 0;
    return;
}

void ParabolicCurve::SetConstant(dReal x0, dReal t)
{
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    if( t < 0 ) {
        t = 0;
    }

    std::vector<Ramp> ramps(1);
    ramps[0].Initialize(0, 0, t, x0);
    Initialize(ramps);
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
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
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

                 a* = argmin_a C(a) = (v0 + at - v1)^2 + (x0 + v0*t + 0.5*a*t^2)^2.

         >>>> import sympy as sp
         >>>> x0, x1, v0, v1, a, t = sp.symbols(['x0', 'x1', 'v0', 'v1', 'a', 't'])
         >>>> C = (v0 + a*t - v1)**2 + (x0 + v0*t + 0.5*a*t*t - x1)**2

         We can see that C(a) is a *convex* parabola, i.e., the coefficient of a^2 is strictly
         positive (given that t > 0). Therefore, it always has a minimum and the minimum is at

                 a* = -(2*v0 + t*x0 + 2*v0*t^2)/(0.5*t^3 + 2*t).

         >>>> sp.solve(sp.diff(C, a), a)
     */
    dReal tSqr = t*t;
    dReal a = -(v0*tSqr + t*(x0 - x1) + 2*(v0 - v1))/(t*(0.5*tSqr + 2));
    std::vector<Ramp> ramps(1);

    // Directly assign values to avoid recalculation.
    ramps[0].x0 = x0;
    ramps[0].x1 = x1;
    ramps[0].v0 = v0;
    ramps[0].v1 = v1;
    ramps[0].duration = t;
    ramps[0].d = x1 - x0;
    ramps[0].a = a;
    Initialize(ramps);
    return;
}

void ParabolicCurve::SetZeroDuration(dReal x0, dReal v0)
{
    std::vector<Ramp> ramps(1);
    ramps[0].Initialize(v0, 0, 0, x0);
    Initialize(ramps);
    return;
}

void ParabolicCurve::Swap(ParabolicCurve& anotherCurve)
{
    _ramps.swap(anotherCurve._ramps);
    _switchpointsList.swap(anotherCurve._switchpointsList);
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
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, _duration + g_fRampEpsilon);

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

        remCurve._switchpointsList.resize(_switchpointsList.size() - index);
        std::copy(_switchpointsList.begin() + index, _switchpointsList.end(), remCurve._switchpointsList.begin());
        remCurve._switchpointsList[0] = 0;
        for (std::vector<dReal>::iterator itsw = remCurve._switchpointsList.begin() + 1; itsw != remCurve._switchpointsList.end(); ++itsw) {
            // Remove duration offset
            *itsw -= _switchpointsList[index];
        }
        remCurve._duration = remCurve._switchpointsList.back();

        _ramps.resize(index);
        _switchpointsList.resize(index + 1);
        _d = GetX1() - GetX0();
        _duration = _switchpointsList.back();
        return;
    }
    else {
        // Note that index will always be strictly less than _ramps.size().
        remCurve._ramps.resize(_ramps.size() - index);
        std::copy(_ramps.begin() + index, _ramps.end(), remCurve._ramps.begin());
        remCurve._ramps.front().TrimFront(remainder);
        remCurve._d = remCurve.GetX1() - remCurve.GetX0();

        remCurve._switchpointsList.resize(_switchpointsList.size() - index);
        std::copy(_switchpointsList.begin() + index, _switchpointsList.end(), remCurve._switchpointsList.begin());
        remCurve._switchpointsList[0] = 0;
        dReal durationOffset = _switchpointsList[index] + remainder;
        for (std::vector<dReal>::iterator itsw = remCurve._switchpointsList.begin() + 1; itsw != remCurve._switchpointsList.end(); ++itsw) {
            // Remove duration offset
            *itsw -= durationOffset;
        }
        remCurve._duration = remCurve._switchpointsList.back();

        _ramps.resize(index + 1);
        _ramps.back().TrimBack(remainder);
        _switchpointsList.resize(index + 2);
        _switchpointsList.back() = _switchpointsList[_switchpointsList.size() - 2] + remainder;
        _d = GetX1() - GetX0();
        _duration = _switchpointsList.back();
    }
}

void ParabolicCurve::TrimFront(dReal t)
{
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, _duration + g_fRampEpsilon);

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

    std::vector<Ramp> rightHalf(_ramps.size() - index);
    std::copy(_ramps.begin() + index, _ramps.end(), rightHalf.begin());
    rightHalf.front().TrimFront(remainder);
    Initialize(rightHalf);
    return;
}

void ParabolicCurve::TrimBack(dReal t)
{
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, _duration + g_fRampEpsilon);

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
        _switchpointsList.resize(index + 1);
    }
    else {
        _ramps.resize(index + 1);
        _ramps.back().TrimBack(remainder);
        _switchpointsList.resize(index + 2);
        _switchpointsList.back() = _switchpointsList[_switchpointsList.size() - 2] + remainder;
    }
    _d = GetX1() - GetX0();
    _duration = _switchpointsList.back();
    return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// RampND
RampND::RampND(size_t ndof)
{
    OPENRAVE_ASSERT_OP(ndof, >, 0);
    _ndof = ndof;
    _data.resize(6*_ndof + 1);
    constraintChecked = false;
}

RampND::RampND(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect, const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect, const std::vector<dReal>& aVect, const std::vector<dReal>& dVect, dReal t)
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
    if( dVect.size() > 0 ) {
        OPENRAVE_ASSERT_OP(dVect.size(), ==, _ndof);
    }

    _data.resize(6*_ndof + 1);

    std::copy(x0Vect.begin(), x0Vect.end(), _data.begin());
    std::copy(x1Vect.begin(), x1Vect.end(), _data.begin() + _ndof);
    std::copy(v0Vect.begin(), v0Vect.end(), _data.begin() + 2*_ndof);
    std::copy(v1Vect.begin(), v1Vect.end(), _data.begin() + 3*_ndof);

    if( aVect.size() == 0 ) {
        if( t == 0 ) {
            std::fill(_data.begin() + 4*_ndof, _data.begin() + 5*_ndof, 0);
        }
        else {
            // Calculating accelerations using the same procedure as in ParabolicCurve::SetSegment.
            dReal tSqr = t*t;
            for (size_t idof = 0; idof < _ndof; ++idof) {
                _data[4*_ndof + idof] = -(v0Vect[idof]*tSqr + t*(x0Vect[idof] - x1Vect[idof]) + 2*(v0Vect[idof] - v1Vect[idof]))/(t*(0.5*tSqr + 2));
            }
        }
    }
    else {
        std::copy(aVect.begin(), aVect.end(), _data.begin() + 4*_ndof);
    }

    if( dVect.size() == 0 ) {
        if( t == 0 ) {
            std::fill(_data.begin() + 5*_ndof, _data.begin() + 6*_ndof, 0);
        }
        else {
            for (size_t idof = 0; idof < _ndof; ++idof) {
                _data[5*_ndof + idof] = x1Vect[idof] - x0Vect[idof];
            }
        }
    }
    else {
        std::copy(dVect.begin(), dVect.end(), _data.begin() + 5*_ndof);
    }

    _data.back() = t;

    constraintChecked = false;
}

void RampND::EvalPos(dReal t, std::vector<dReal>& xVect) const
{
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, _data.back() + g_fRampEpsilon);

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
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, _data.back() + g_fRampEpsilon);

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
    _data.resize(6*_ndof + 1);
    std::fill(_data.begin(), _data.end(), 0);
}

void RampND::Initialize(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect, const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect, const std::vector<dReal>& aVect, const std::vector<dReal>& dVect, dReal t)
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
    if( dVect.size() > 0 ) {
        OPENRAVE_ASSERT_OP(dVect.size(), ==, _ndof);
    }

    _data.resize(6*_ndof + 1);

    std::copy(x0Vect.begin(), x0Vect.end(), _data.begin());
    std::copy(x1Vect.begin(), x1Vect.end(), _data.begin() + _ndof);
    std::copy(v0Vect.begin(), v0Vect.end(), _data.begin() + 2*_ndof);
    std::copy(v1Vect.begin(), v1Vect.end(), _data.begin() + 3*_ndof);

    if( aVect.size() == 0 ) {
        if( t == 0 ) {
            std::fill(_data.begin() + 4*_ndof, _data.begin() + 5*_ndof, 0);
        }
        else {
            // Calculating accelerations using the same procedure as in ParabolicCurve::SetSegment.
            dReal tSqr = t*t;
            for (size_t idof = 0; idof < _ndof; ++idof) {
                _data[4*_ndof + idof] = -(v0Vect[idof]*tSqr + t*(x0Vect[idof] - x1Vect[idof]) + 2*(v0Vect[idof] - v1Vect[idof]))/(t*(0.5*tSqr + 2));
            }
        }
    }
    else {
        std::copy(aVect.begin(), aVect.end(), _data.begin() + 4*_ndof);
    }

    if( dVect.size() == 0 ) {
        if( t == 0 ) {
            std::fill(_data.begin() + 5*_ndof, _data.begin() + 6*_ndof, 0);
        }
        else {
            for (size_t idof = 0; idof < _ndof; ++idof) {
                _data[5*_ndof + idof] = x1Vect[idof] - x0Vect[idof];
            }
        }
    }
    else {
        std::copy(dVect.begin(), dVect.end(), _data.begin() + 5*_ndof);
    }

    _data.back() = t;

    constraintChecked = false;
}

void RampND::SetConstant(const std::vector<dReal>& xVect, const dReal t)
{
    OPENRAVE_ASSERT_OP(xVect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    size_t offset = _ndof;
    std::copy(xVect.begin(), xVect.end(), _data.begin()); // x0Vect
    std::copy(xVect.begin(), xVect.end(), _data.begin() + _ndof); // x1Vect
    offset += _ndof;
    std::fill(_data.begin() + offset, _data.end() - 1, 0); // v0Vect, v1Vect, aVect, dVect
    _data.back() = t; // duration
    return;
}

void RampND::SetInitialPosition(const std::vector<dReal>& xVect)
{
    OPENRAVE_ASSERT_OP(xVect.size(), ==, _ndof);
    std::copy(xVect.begin(), xVect.end(), _data.begin()); // x0Vect
    for (size_t idof = 0; idof < _ndof; ++idof) {
        SetX1At(idof) = GetX0At(idof) + GetDAt(idof);
    }
    return;
}

void RampND::Cut(dReal t, RampND& remRampND)
{
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, _data.back() + g_fRampEpsilon);
    if( remRampND._ndof != _ndof ) {
        remRampND.Initialize(_ndof);
    }

    remRampND.constraintChecked = constraintChecked;

    if( t <= 0 || t >= _data.back() ) {
        std::copy(_data.begin() + _ndof, _data.begin() + 2*_ndof, remRampND._data.begin()); // position
        std::copy(_data.begin() + _ndof, _data.begin() + 2*_ndof, remRampND._data.begin() + _ndof);

        std::copy(_data.begin() + 3*_ndof, _data.begin() + 4*_ndof, remRampND._data.begin() + 2*_ndof); // velocity
        std::copy(_data.begin() + 3*_ndof, _data.begin() + 4*_ndof, remRampND._data.begin() + 3*_ndof);

        std::copy(_data.begin() + 4*_ndof, _data.begin() + 5*_ndof, remRampND._data.begin() + 4*_ndof); // acceleration
        std::fill(remRampND._data.begin() + 5*_ndof, remRampND._data.begin() + 6*_ndof, 0); // displacement

        remRampND._data.back() = 0;

        if( t <= 0 ) {
            _data.swap(remRampND._data);
        }
        return;
    }
    else {
        std::vector<dReal> temp(_ndof); // temporary value holder
        EvalPos(t, temp);
        std::copy(temp.begin(), temp.end(), _data.begin() + _ndof);
        std::copy(temp.begin(), temp.end(), remRampND._data.begin());

        EvalVel(t, temp); // changing of x1 does not affect velocity calculation
        std::copy(temp.begin(), temp.end(), _data.begin() + 3*_ndof);
        std::copy(temp.begin(), temp.end(), remRampND._data.begin() + 2*_ndof);

        EvalAcc(temp);
        std::copy(temp.begin(), temp.end(), remRampND._data.begin() + 4*_ndof);

        for (size_t idof = 0; idof < _ndof; ++idof) {
            remRampND._data[5*_ndof + idof] = remRampND.GetX1At(idof) - remRampND.GetX0At(idof);
        }

        remRampND._data.back() = _data.back() - t;
        _data.back() = t;
        return;
    }
}

void RampND::TrimFront(dReal t)
{
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, _data.back() + g_fRampEpsilon);

    if( t <= 0 ) {
        return;
    }
    else if( t >= _data.back() ) {
        std::copy(_data.begin() + _ndof, _data.begin() + 2*_ndof, _data.begin()); // replace x0 by x1
        std::copy(_data.begin() + 3*_ndof, _data.begin() + 4*_ndof, _data.begin() + 2*_ndof); // replace v0 by v1
        std::fill(_data.begin() + 5*_ndof, _data.begin() + 6*_ndof, 0); // displacement
        _data.back() = 0; // duration
        return;
    }
    else {
        std::vector<dReal> temp(_ndof); // temporary value holder
        EvalPos(t, temp);
        std::copy(temp.begin(), temp.end(), _data.begin()); // update x0

        EvalVel(t, temp); // changing of x0 does not affect velocity calculation
        std::copy(temp.begin(), temp.end(), _data.begin() + 2*_ndof); // update v0

        // update d
        for (size_t idof = 0; idof < _ndof; ++idof) {
            _data[5*_ndof + idof] = GetX1At(idof) - GetX0At(idof);
        }

        _data.back() = _data.back() - t;
        return;
    }
}

void RampND::TrimBack(dReal t)
{
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, _data.back() + g_fRampEpsilon);

    if( t <= 0 ) {
        std::copy(_data.begin(), _data.begin() + _ndof, _data.begin() + _ndof); // replace x1 by x0
        std::copy(_data.begin() + 2*_ndof, _data.begin() + 3*_ndof, _data.begin() + 3*_ndof); // replace v1 by v0
        std::fill(_data.begin() + 5*_ndof, _data.begin() + 6*_ndof, 0); // displacement
        _data.back() = 0; // duration
        return;
    }
    else if( t >= _data.back() ) {
        return;
    }
    else {
        std::vector<dReal> temp(_ndof); // temporary value holder
        EvalPos(t, temp);
        std::copy(temp.begin(), temp.end(), _data.begin() + _ndof); // update x1

        EvalVel(t, temp); // changing of x0 does not affect velocity calculation
        std::copy(temp.begin(), temp.end(), _data.begin() + 3*_ndof); // update v1

        // update d
        for (size_t idof = 0; idof < _ndof; ++idof) {
            _data[5*_ndof + idof] = GetX1At(idof) - GetX0At(idof);
        }

        _data.back() = t;
        return;
    }
}

void RampND::Serialize(std::ostream& O) const
{
    std::string separator = "";
    for (size_t i = 0; i < _data.size(); ++i) {
        O << separator << _data[i];
        separator = " ";
    }
    O << "\n";
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// ParabolicPath
void ParabolicPath::AppendRampND(RampND& rampndIn)
{
    if( _rampnds.capacity() < _rampnds.size() + 1 ) {
        _rampnds.reserve(_rampnds.size() + 1);
    }
    _rampnds.push_back(rampndIn);

    if( _switchpointsList.capacity() < _rampnds.size() + 1 ) {
        _switchpointsList.reserve(_rampnds.size() + 1);
    }
    _switchpointsList.push_back(_duration + rampndIn.GetDuration());

    _duration = _switchpointsList.back();
}

void ParabolicPath::EvalPos(dReal t, std::vector<dReal>& xVect) const
{
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, _duration + g_fRampEpsilon);
}

void ParabolicPath::EvalVel(dReal t, std::vector<dReal>& vVect) const
{
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, _duration + g_fRampEpsilon);
}

void ParabolicPath::EvalAcc(dReal t, std::vector<dReal>& aVect) const
{
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, _duration + g_fRampEpsilon);
}

void ParabolicPath::FindRampNDIndex(dReal t, int& index, dReal& remainder) const
{
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, _duration + g_fRampEpsilon);

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
        std::vector<dReal>::const_iterator it = _switchpointsList.begin();
        while ( it != _switchpointsList.end() && t >= *it ) {
            index_++;
            it++;
        }
        OPENRAVE_ASSERT_OP(index_, <, (int)_switchpointsList.size());
        index = index_ - 1;
        remainder = t - *(it - 1);
        return;
    }
}

void ParabolicPath::Initialize(const RampND& rampndIn)
{
    _rampnds.resize(0);
    _rampnds.reserve(1);
    _rampnds.push_back(rampndIn);

    _switchpointsList.resize(2);
    _switchpointsList[0] = 0;
    _switchpointsList[1] = rampndIn.GetDuration();

    _duration = _switchpointsList.back();
}

void ParabolicPath::ReplaceSegment(dReal t0, dReal t1, const std::vector<RampND>& rampndVect)
{
    OPENRAVE_ASSERT_OP(t0, <, t1);
    OPENRAVE_ASSERT_OP(rampndVect.size(), >, 0);
    if( t0 == 0 && t1 == _duration ) {
        _rampnds = rampndVect;
        _duration = 0;
        _switchpointsList.resize(0);
        _switchpointsList.reserve(rampndVect.size() + 1);
        _switchpointsList.push_back(_duration);
        for (size_t irampnd = 0; irampnd < rampndVect.size(); ++irampnd) {
            _duration += _rampnds[irampnd].GetDuration();
            _switchpointsList.push_back(_duration);
        }
        return;
    }

    int index0, index1;
    dReal rem0, rem1;
    FindRampNDIndex(t0, index0, rem0);
    FindRampNDIndex(t1, index1, rem1);

    /*
       Idea: First resize _rampnds to prepare for inserting rampndVect into it. Imaging dividing
       _rampnds into three segments where the middle part is to be replaced by rampndVect. Next move
       the right segment towards the end of _rampnds. Finally, we replace the middle segment with
       rampndVect.
     */

    // Carefully resize the container
    size_t prevSize = _rampnds.size();
    size_t leftPartLength = rem0 == 0 ? index0 : (index0 + 1);
    size_t rightPartLength = t1 == _duration ? 0 : (prevSize - index1);
    size_t newSize = leftPartLength + rampndVect.size() + rightPartLength;

    if( prevSize < newSize ) {
        // The new size is greater than the current value so we can resize the container right away.
        _rampnds.resize(newSize);
        for (size_t irampnd = 0; irampnd < rightPartLength; ++irampnd) {
            _rampnds[newSize - 1 - irampnd] = _rampnds[prevSize - 1 - irampnd];
        }

        _rampnds[index0].TrimBack(rem0);
        if( rightPartLength > 0 ) {
            _rampnds[newSize - rightPartLength].TrimFront(rem1);
        }
        std::copy(rampndVect.begin(), rampndVect.end(), _rampnds.begin() + index0 + 1);
    }
    else if( prevSize == newSize ) {
        // No resizing required.
        _rampnds[index0].TrimBack(rem0);
        if( rightPartLength > 0 ) {
            _rampnds[index1].TrimFront(rem1);
        }
        std::copy(rampndVect.begin(), rampndVect.end(), _rampnds.begin() + index0 + 1);
    }
    else {
        // The new size is less than the current value so we need to move RampNDs first before
        // resizing.
        for (size_t irampnd = 0; irampnd < rightPartLength; ++irampnd) {
            _rampnds[newSize - rightPartLength + irampnd] = _rampnds[prevSize - rightPartLength + irampnd];
        }
        _rampnds.resize(newSize);

        _rampnds[index0].TrimBack(rem0);
        if( rightPartLength > 0 ) {
            _rampnds[newSize - rightPartLength].TrimFront(rem1);
        }
        std::copy(rampndVect.begin(), rampndVect.end(), _rampnds.begin() + index0 + 1);
    }

    _UpdateMembers();
}

void ParabolicPath::Serialize(std::ostream& O) const
{
    O << std::setprecision(g_nPrec);
    for (std::vector<RampND>::const_iterator itrampnd = _rampnds.begin(); itrampnd != _rampnds.end(); ++itrampnd) {
        itrampnd->Serialize(O);
    }
    return;
}

void ParabolicPath::_UpdateMembers()
{
    _switchpointsList.resize(0);
    _switchpointsList.reserve(_rampnds.size() + 1);
    _switchpointsList.push_back(0);

    _duration = 0;
    for (size_t irampnd = 0; irampnd < _rampnds.size(); ++irampnd) {
        _duration += _rampnds[irampnd].GetDuration();
        _switchpointsList.push_back(_duration);
    }
}

} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
