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
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);

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

dReal Ramp::EvalAcc(dReal t) const
{
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(t, <=, duration + g_fRampEpsilon);

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
    OPENRAVE_ASSERT_OP(t, >=, -g_fRampEpsilon);

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
        remRamp.Copy(this);
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
ParaboliCurve::ParabolicCurve(std::vector<Ramp>&rampsIn)
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
        _ramps.reserve(_ramp.size() + curve._ramps.size());
    }
    if( _switchpointsList.capacity() < _switchpointsList.size() + curve._switchpointsList.size() - 1 ) {
        _switchpointsList.reserve(_switchpointsList.size() + curve._switchpointsList.size() - 1);
    }

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
        while( it != _switchpointsList.end() && t > *it) {
            ++index_;
            ++it;
        }
        OPENRAVE_ASSERT_OP(index, <, (int)switchpointsList.size());
        index = index_;
        remainder = t - *it;
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
    for (std::vector<Ramp>::const_iterator itramp = _ramps.begin(); itramp != _ramps.end(); ++itramp) {
        itramp->SetInitialValue(newx0);
        newx0 += itramp->x1;
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
    _ramp.swap(anotherCurve._ramps);
    _switchpointsList.swap(anotherCurve._switchpointsList);
    Swap(_d, anotherCurve._d);
    Swap(_duration, anotherCurve._duration);
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
    else if( t >= duration ) {
        SetZeroDuration(GetX1(), GetV1());
        return;
    }

    int index;
    dReal remainder;
    FindRampIndex(t, index, remainder);

    std::vector<Ramp> rightHalf(ramps.size() - index);
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
    else if( t >= duration ) {
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
// 

} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
