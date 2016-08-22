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

////////////////////////////////////////////////////////////////////////////////
// Ramp
Ramp::Ramp(dReal v0_, dReal a_, dReal dur_, dReal x0_)  {
    BOOST_ASSERT(dur_ >= -epsilon);

    v0 = v0_;
    a = a_;
    duration = dur_;
    x0 = x0_;

    v1 = v0 + (a*duration);
    d = duration*(v0 + 0.5*a*duration);
    x1 = x0 + d;
}

dReal Ramp::EvalPos(dReal t) const {
    BOOST_ASSERT(t >= -epsilon);
    BOOST_ASSERT(t <= duration + epsilon);
    if (t <= 0) {
        return x0;
    }
    else if (t >= duration) {
        return x1;
    }

    return t*(v0 + 0.5*a*t) + x0;
}

dReal Ramp::EvalVel(dReal t) const {
    BOOST_ASSERT(t >= -epsilon);
    BOOST_ASSERT(t <= duration + epsilon);
    if (t <= 0) {
        return v0;
    }
    else if (t >= duration) {
        return v1;
    }

    return v0 + (a*t);
}

dReal Ramp::EvalAcc(dReal t) const {
    BOOST_ASSERT(t >= -epsilon);
    BOOST_ASSERT(t <= duration + epsilon);
    return a;
}

void Ramp::GetPeaks(dReal& bmin, dReal& bmax) const {
    GetPeaks(0, duration, bmin, bmax);
    return;
}

void Ramp::GetPeaks(dReal ta, dReal tb, dReal& bmin, dReal& bmax) const {
    if (ta > tb) {
        GetPeaks(tb, ta, bmin, bmax);
    }

    if (ta < 0) {
        ta = 0;
    }
    if (tb <= 0) {
        bmin = x0;
        bmax = x0;
        return;
    }

    if (tb > duration) {
        tb = duration;
    }
    if (ta >= duration) {
        bmin = x1;
        bmax = x1;
        return;
    }

    if (FuzzyZero(a, epsilon)) {
        if (v0 > 0) {
            bmin = EvalPos(ta);
            bmax = EvalPos(tb);
        }
        else {
            bmin = EvalPos(tb);
            bmax = EvalPos(ta);
        }
        return;
    }
    else if (a > 0) {
        bmin = EvalPos(ta);
        bmax = EvalPos(tb);
    }
    else {
        bmin = EvalPos(tb);
        bmax = EvalPos(ta);
    }

    dReal tDeflection = -v0/a;
    if ((tDeflection <= ta) || (tDeflection >= tb)) {
        return;
    }

    dReal xDeflection = EvalPos(tDeflection);
    bmin = Min(bmin, xDeflection);
    bmax = Max(bmax, xDeflection);
    return;
}

void Ramp::Initialize(dReal v0_, dReal a_, dReal dur_, dReal x0_) {
    BOOST_ASSERT(dur_ >= -epsilon);

    v0 = v0_;
    a = a_;
    duration = dur_;
    x0 = x0_;

    v1 = v0 + (a*duration);
    d = duration*(v0 + 0.5*a*duration);
    x1 = x0 + d;
    return;
}

void Ramp::PrintInfo(std::string name) const {
    std::cout << "Ramp information: " << name << std::endl;
    std::cout << str(boost::format("  v0 = %.15e")%v0) << std::endl;
    std::cout << str(boost::format("   a = %.15e")%a) << std::endl;
    std::cout << str(boost::format("   t = %.15e")%duration) << std::endl;
    std::cout << str(boost::format("  x0 = %.15e")%x0) << std::endl;
    std::cout << str(boost::format("  v1 = %.15e")%v1) << std::endl;
    std::cout << str(boost::format("   d = %.15e")%d) << std::endl;
    std::cout << str(boost::format("  x1 = %.15e")%x1) << std::endl;
}

void Ramp::UpdateDuration(dReal newDuration) {
    BOOST_ASSERT(newDuration >= -epsilon);
    if (newDuration < 0) {
        newDuration = 0;
    }
    // Update the members accordinly
    duration = newDuration;
    v1 = v0 + (a*duration);
    d = duration*(v0 + 0.5*a*duration);
    x1 = x0 + d;
    return;
}

void Ramp::SetInitialValue(dReal newx0) {
    x0 = newx0;
    x1 = x0 + d;
    return;
}

void Ramp::Cut(dReal t, Ramp &remRamp) {
    BOOST_ASSERT(t >= -epsilon);
    BOOST_ASSERT(t <= duration + epsilon);
    if (t <= 0) {
        remRamp.Initialize(v0, a, duration, x0);
        Initialize(v0, 0, 0, x0);
        return;
    }
    else if (t >= duration) {
        remRamp.Initialize(v1, 0, 0, x1);
        return;
    }

    dReal remRampDuration = duration - t;
    UpdateDuration(t);
    remRamp.Initialize(v1, a, remRampDuration, x1);
    return;
}

void Ramp::TrimFront(dReal t) {
    // Trim front, keep back
    BOOST_ASSERT(t >= -epsilon);
    BOOST_ASSERT(t <= duration + epsilon);
    if (t <= 0) {
        return;
    }
    else if (t >= duration) {
        Initialize(v1, 0, 0, x1);
        return;
    }

    dReal remDuration = duration - t;
    dReal newx0 = EvalPos(t);
    dReal newv0 = EvalVel(t);

    Initialize(newv0, a, remDuration, newx0);
    return;
}

void Ramp::TrimBack(dReal t) {
    // Trim back, keep front
    BOOST_ASSERT(t >= -epsilon);
    BOOST_ASSERT(t <= duration + epsilon);
    if (t <= 0) {
        Initialize(v0, 0, 0, x0);
        return;
    }
    else if (t >= duration) {
        return;
    }

    UpdateDuration(t);
    return;
}

////////////////////////////////////////////////////////////////////////////////
// ParabolicCurve
ParabolicCurve::ParabolicCurve(std::vector<Ramp> rampsIn) {
    BOOST_ASSERT(!rampsIn.empty());

    ramps.resize(0);
    ramps.reserve(rampsIn.size());
    switchpointsList.resize(0);
    switchpointsList.reserve(rampsIn.size() + 1);
    d = 0.0;
    duration = 0.0;
    switchpointsList.push_back(duration);

    for (std::size_t i = 0; i != rampsIn.size(); ++i) {
        ramps.push_back(rampsIn[i]);
        d = d + ramps.back().d;
        duration = duration + ramps[i].duration;
        switchpointsList.push_back(duration);
    }
    v0 = rampsIn.front().v0;
    v1 = rampsIn.back().v1;
    SetInitialValue(rampsIn[0].x0);
}

void ParabolicCurve::Append(ParabolicCurve curve) {
    std::size_t prevLength = 0;
    std::size_t sz = curve.ramps.size();
    if (ramps.size() == 0) {
        switchpointsList.reserve(sz + 1);
        d = 0.0;
        duration = 0.0;
        switchpointsList.push_back(duration);
        x0 = curve.ramps[0].x0;
        v0 = curve.ramps[0].v0;
    }
    prevLength = ramps.size();
    ramps.reserve(prevLength + sz);

    for (std::size_t i = 0; i != sz; ++i) {
        ramps.push_back(curve.ramps[i]);
        d = d + ramps.back().d;
        duration = duration + ramps.back().duration;
        switchpointsList.push_back(duration);
    }
    v1 = curve.v1;
    SetInitialValue(x0);
    return;
}

void ParabolicCurve::Reset() {
    x0 = 0;
    x1 = 0;
    duration = 0;
    d = 0;
    v0 = 0;
    v1 = 0;
    switchpointsList.clear();
    ramps.clear();
    return;
}

void ParabolicCurve::SetInitialValue(dReal newx0) {
    x0 = newx0;
    size_t nRamps = ramps.size();
    for (size_t i = 0; i < nRamps; ++i) {
        ramps[i].SetInitialValue(newx0);
        newx0 = ramps[i].x1;
    }
    x1 = x0 + d;
    return;
}

void ParabolicCurve::FindRampIndex(dReal t, int& index, dReal& remainder) const {
    BOOST_ASSERT(t >= -epsilon);
    BOOST_ASSERT(t <= duration + epsilon);
    if (t <= 0) {
        index = 0;
        remainder = 0;
    }
    else if (t >= duration) {
        index = ((int) ramps.size()) - 1;
        remainder = ramps.back().duration;
    }
    else {
        index = 0;
        // Iterate through switchpointsList
        std::vector<dReal>::const_iterator it = switchpointsList.begin();
        while (it != switchpointsList.end() && t >= *it) {
            index++;
            it++;
        }
        BOOST_ASSERT(index < (int)switchpointsList.size());
        index = index - 1;
        remainder = t - *(it - 1);
    }
    return;
}

void ParabolicCurve::Initialize(std::vector<Ramp> rampsIn) {
    ramps.resize(0);
    ramps.reserve(rampsIn.size());

    switchpointsList.resize(0);
    switchpointsList.reserve(rampsIn.size() + 1);

    d = 0.0;
    duration = 0.0;
    switchpointsList.push_back(duration);
    v0 = rampsIn.front().v0;
    v1 = rampsIn.back().v1;

    for (std::size_t i = 0; i != rampsIn.size(); ++i) {
        ramps.push_back(rampsIn[i]);
        d = d + ramps.back().d;
        duration = duration + ramps[i].duration;
        switchpointsList.push_back(duration);
    }
    SetInitialValue(rampsIn[0].x0);
    return;
}

void ParabolicCurve::PrintInfo(std::string name) const {
    std::cout << "ParabolicCurve information: " << name << std::endl;
    std::cout << str(boost::format("  This parabolic curve consists of %d ramps")%(ramps.size())) << std::endl;
    std::cout << str(boost::format("  v0 = %.15e")%(ramps[0].v0)) << std::endl;
    std::cout << str(boost::format("   t = %.15e")%duration) << std::endl;
    std::cout << str(boost::format("  x0 = %.15e")%x0) << std::endl;
    std::cout << str(boost::format("  x1 = %.15e")%x1) << std::endl;
    std::cout << str(boost::format("   d = %.15e")%d) << std::endl;
    std::string swList = GenerateStringFromVector(switchpointsList);
    swList = "  Switch points = " + swList;
    std::cout << swList << std::endl;
}

dReal ParabolicCurve::EvalPos(dReal t) const {
    BOOST_ASSERT(t >= -epsilon);
    BOOST_ASSERT(t <= duration + epsilon);
    if (t <= 0) {
        return x0;
    }
    else if (t >= duration) {
        return x1;
    }

    int index;
    dReal remainder;
    FindRampIndex(t, index, remainder);
    return ramps[index].EvalPos(remainder);
}

dReal ParabolicCurve::EvalVel(dReal t) const {
    BOOST_ASSERT(t >= -epsilon);
    BOOST_ASSERT(t <= duration + epsilon);
    if (t <= 0) {
        return v0;
    }
    else if (t >= duration) {
        return v1;
    }

    int index;
    dReal remainder;
    FindRampIndex(t, index, remainder);
    return ramps[index].EvalVel(remainder);
}

dReal ParabolicCurve::EvalAcc(dReal t) const {
    BOOST_ASSERT(t >= -epsilon);
    BOOST_ASSERT(t <= duration + epsilon);
    if (t <= 0) {
        return ramps[0].a;
    }
    else if (t >= duration) {
        return ramps.back().a;
    }

    int index;
    dReal remainder;
    FindRampIndex(t, index, remainder);
    return ramps[index].a;
}

void ParabolicCurve::GetPeaks(dReal& bmin, dReal& bmax) const {
    GetPeaks(0, duration, bmin, bmax);
    return;
}

void ParabolicCurve::GetPeaks(dReal ta, dReal tb, dReal& bmin, dReal& bmax) const {
    bmin = inf;
    bmax = -inf;
    dReal temp1, temp2;
    for (std::vector<Ramp>::const_iterator it = ramps.begin(); it != ramps.end(); ++it) {
        it->GetPeaks(ta, tb, temp1, temp2);
        if (temp1 < bmin) {
            bmin = temp1;
        }
        if (temp2 > bmax) {
            bmax = temp2;
        }
    }
    RAMP_OPTIM_ASSERT(bmin < inf);
    RAMP_OPTIM_ASSERT(bmax > -inf);
    return;
}

void ParabolicCurve::SetConstant(dReal _x0, dReal t) {
    if (t < 0) {
        RAMP_OPTIM_WARN("Invalid time given: t = %.15e", t);
    }
    RAMP_OPTIM_ASSERT(t >= 0);

    Ramp ramp(0, 0, t, _x0);
    std::vector<Ramp> _ramps(1);
    _ramps[0] = ramp;
    Initialize(_ramps);
    return;
}

void ParabolicCurve::SetSegment(dReal _x0, dReal _x1, dReal _v0, dReal _v1, dReal t) {
    if (t <= 0) {
        RAMP_OPTIM_WARN("Invalid time given: t = %.15e", t);
    }
    RAMP_OPTIM_ASSERT(t > 0);

    Ramp ramp(_v0, (_v1 - _v0)/t, t, _x0);
    RAMP_OPTIM_ASSERT(FuzzyEquals(ramp.x1, _x1, epsilon));

    std::vector<Ramp> _ramps(1);
    _ramps[0] = ramp;
    Initialize(_ramps);
    return;
}

void ParabolicCurve::SetZeroDuration(dReal _x0, dReal _v0) {
    Ramp ramp(_v0, 0, 0, _x0);
    std::vector<Ramp> _ramps(1);
    _ramps[0] = ramp;
    Initialize(_ramps);
    return;
}

void ParabolicCurve::Cut(dReal t, ParabolicCurve &remCurve) {
    BOOST_ASSERT(t >= -epsilon);
    BOOST_ASSERT(t <= duration + epsilon);

    if (t <= 0) {
        remCurve.Initialize(ramps);
        SetZeroDuration(x0, v0);
        return;
    }
    else if (t >= duration) {
        remCurve.SetZeroDuration(x1, v1);
        return;
    }

    int index;
    dReal remainder;
    FindRampIndex(t, index, remainder);

    std::vector<Ramp> leftHalf(index + 1);
    std::vector<Ramp> rightHalf(ramps.size() - index);

    std::copy(ramps.begin(), ramps.begin() + index + 1, leftHalf.begin());
    std::copy(ramps.begin() + index, ramps.end(), rightHalf.begin());

    leftHalf.back().TrimBack(remainder);
    rightHalf.front().TrimFront(remainder);

    Initialize(leftHalf);
    remCurve.Initialize(rightHalf);
    return;
}

void ParabolicCurve::TrimFront(dReal t) {
    BOOST_ASSERT(t >= -epsilon);
    BOOST_ASSERT(t <= duration + epsilon);

    if (t <= 0) {
        return;
    }
    else if (t >= duration) {
        SetZeroDuration(x1, v1);
        return;
    }

    int index;
    dReal remainder;
    FindRampIndex(t, index, remainder);

    std::vector<Ramp> rightHalf(ramps.size() - index);
    std::copy(ramps.begin() + index, ramps.end(), rightHalf.begin());
    rightHalf.front().TrimFront(remainder);

    Initialize(rightHalf);
    return;
}

void ParabolicCurve::TrimBack(dReal t) {
    BOOST_ASSERT(t >= -epsilon);
    BOOST_ASSERT(t <= duration + epsilon);

    if (t <= 0) {
        SetZeroDuration(x0, v0);
        return;
    }
    else if (t >= duration) {
        return;
    }

    int index;
    dReal remainder;
    FindRampIndex(t, index, remainder);

    std::vector<Ramp> leftHalf(index + 1);

    std::copy(ramps.begin(), ramps.begin() + index + 1, leftHalf.begin());
    leftHalf.back().TrimBack(remainder);

    Initialize(leftHalf);
    return;
}


////////////////////////////////////////////////////////////////////////////////
// paraboliccurvesnd
ParabolicCurvesND::ParabolicCurvesND(std::vector<ParabolicCurve> curvesIn) {
    BOOST_ASSERT(!curvesIn.empty());

    ndof = curvesIn.size();
    // Here we need to check whether every curve has (roughly) the same duration
    ///////////////////
    dReal minDur = curvesIn[0].duration;
    for (std::vector<ParabolicCurve>::const_iterator it = curvesIn.begin() + 1; it != curvesIn.end(); ++it) {
        BOOST_ASSERT(FuzzyEquals(it->duration, minDur, epsilon));
        minDur = Min(minDur, it->duration);
    }
    // Need to think about how to handle discrepancies in durations.

    curves = curvesIn;
    duration = minDur;

    x0Vect.reserve(ndof);
    x1Vect.reserve(ndof);
    dVect.reserve(ndof);
    v0Vect.reserve(ndof);
    v1Vect.reserve(ndof);
    for (size_t i = 0; i < ndof; ++i) {
        x0Vect.push_back(curves[i].x0);
        x1Vect.push_back(curves[i].x1);
        v0Vect.push_back(curves[i].v0);
        v1Vect.push_back(curves[i].v1);
        dVect.push_back(curves[i].d);
    }

    // Manage switch points
    switchpointsList = curves[0].switchpointsList;
    std::vector<dReal>::iterator it;
    for (size_t i = 0; i < ndof; i++) {
        for (size_t j = 1; j != curves[i].switchpointsList.size() - 1; ++j) {
            // Iterate from the second element to the second last element (the first and the last
            // switch points should be the same for every DOF)
            it = std::lower_bound(switchpointsList.begin(), switchpointsList.end(), curves[i].switchpointsList[j]);
            if (it == switchpointsList.end()) {
                if (!FuzzyEquals(curves[i].switchpointsList[j], *(it - 1), epsilon)) {
                    switchpointsList.insert(it, curves[i].switchpointsList[j]);
                }
            }
            else if (it == switchpointsList.begin()) {
                if (!FuzzyEquals(curves[i].switchpointsList[j], *it, epsilon)) {
                    switchpointsList.insert(it, curves[i].switchpointsList[j]);
                }
            }
            else {
                if (!FuzzyEquals(curves[i].switchpointsList[j], *it, epsilon) && !FuzzyEquals(curves[i].switchpointsList[j], *(it - 1), epsilon)) {
                    // Insert only non-redundant switch points
                    switchpointsList.insert(it, curves[i].switchpointsList[j]);
                }
            }
        }
    }

    constraintCheckedVect.resize(switchpointsList.size() - 1);
    fill(constraintCheckedVect.begin(), constraintCheckedVect.end(), false);
    constraintChecked = false;
}

void ParabolicCurvesND::Append(ParabolicCurvesND curvesnd) {
    BOOST_ASSERT(!curvesnd.IsEmpty());
    // Users need to make sure that the displacement and velocity vectors are continuous

    if (curves.empty()) {
        curves = curvesnd.curves;
        ndof = curves.size();
        duration = curvesnd.duration;
        x0Vect = curvesnd.x0Vect;
        x1Vect = curvesnd.x1Vect;
        dVect = curvesnd.dVect;
        v0Vect = curvesnd.v0Vect;
        v1Vect = curvesnd.v1Vect;
        switchpointsList = curvesnd.switchpointsList;
    }
    else {
        BOOST_ASSERT(curvesnd.ndof == ndof);
        dReal originalDur = duration;
        duration = duration + curvesnd.duration;
        for (size_t i = 0; i < ndof; i++) {
            curves[i].Append(curvesnd.curves[i]);
            v1Vect[i] = curvesnd.curves[i].v1;
            x1Vect[i] = curves[i].x1;
            dVect[i] = dVect[i] + curvesnd.curves[i].d;
        }
        std::vector<dReal> tempSwitchpointsList = curvesnd.switchpointsList;
        // Add every element in tempSwitchpointsList by originalDur
        for (std::vector<dReal>::iterator it = tempSwitchpointsList.begin(); it != tempSwitchpointsList.end(); ++it) {
            *it += originalDur;
        }
        switchpointsList.pop_back(); // remove the last element of switchpointsList since it must be
                                     // the same as the first switch point from tempSwitchpointsList
        switchpointsList.reserve(switchpointsList.size() + tempSwitchpointsList.size());
        switchpointsList.insert(switchpointsList.end(), tempSwitchpointsList.begin(), tempSwitchpointsList.end());

        constraintCheckedVect.insert(constraintCheckedVect.end(), curvesnd.constraintCheckedVect.begin(), curvesnd.constraintCheckedVect.end());

        if (constraintChecked) {
            constraintChecked = constraintChecked && curvesnd.constraintChecked;
        }
    }
    return;
}

void ParabolicCurvesND::Initialize(std::vector<ParabolicCurve> curvesIn) {
    Reset();
    
    ndof = curvesIn.size();
    // Here we need to check whether every curve has (roughly) the same duration
    ///////////////////
    dReal minDur = curvesIn[0].duration;
    for (std::vector<ParabolicCurve>::const_iterator it = curvesIn.begin() + 1; it != curvesIn.end(); ++it) {
        BOOST_ASSERT(FuzzyEquals(it->duration, minDur, epsilon));
        minDur = Min(minDur, it->duration);
    }
    
    curves = curvesIn;
    duration = minDur;

    x0Vect.reserve(ndof);
    x1Vect.reserve(ndof);
    dVect.reserve(ndof);
    v0Vect.reserve(ndof);
    v1Vect.reserve(ndof);
    for (size_t i = 0; i < ndof; ++i) {
        x0Vect.push_back(curves[i].x0);
        x1Vect.push_back(curves[i].x1);
        v0Vect.push_back(curves[i].v0);
        v1Vect.push_back(curves[i].v1);
        dVect.push_back(curves[i].d);
    }

    // Manage switch points (later I will merge this for loop with the one above)
    switchpointsList = curves[0].switchpointsList;
    std::vector<dReal>::iterator it;
    for (size_t i = 0; i < ndof; ++i) {
        for (size_t j = 1; j != curves[i].switchpointsList.size() - 1; ++j) {
            // Iterate from the second element to the second last element (the first and the last
            // switch points should be the same for every DOF)
            it = std::lower_bound(switchpointsList.begin(), switchpointsList.end(), curves[i].switchpointsList[j]);
            if (it == switchpointsList.end()) {
                if (!FuzzyEquals(curves[i].switchpointsList[j], *(it - 1), epsilon)) {
                    switchpointsList.insert(it, curves[i].switchpointsList[j]);
                }
            }
            else if (it == switchpointsList.begin()) {
                if (!FuzzyEquals(curves[i].switchpointsList[j], *it, epsilon)) {
                    switchpointsList.insert(it, curves[i].switchpointsList[j]);
                }
            }
            else {
                if (!FuzzyEquals(curves[i].switchpointsList[j], *it, epsilon) && !FuzzyEquals(curves[i].switchpointsList[j], *(it - 1), epsilon)) {
                    // Insert only non-redundant switch points
                    switchpointsList.insert(it, curves[i].switchpointsList[j]);
                }
            }
        }
    }

    constraintCheckedVect.resize(switchpointsList.size() - 1);
    fill(constraintCheckedVect.begin(), constraintCheckedVect.end(), false);
    constraintChecked = false;
    // std::cout << "INITIALIZATION WITH DURATION = " << duration << std::endl;
    return;
}

void ParabolicCurvesND::SetInitialValues(const std::vector<dReal>& _x0Vect) {
    RAMP_OPTIM_ASSERT(_x0Vect.size() == ndof);

    x0Vect = _x0Vect;
    for (size_t idof = 0; idof < ndof; ++idof) {
        curves[idof].SetInitialValue(x0Vect[idof]);
        x1Vect[idof] = curves[idof].x1;
    }
    return;
}

void ParabolicCurvesND::PrintInfo(std::string name) const {
    std::cout << "ParabolicCurvesND information: " << name << std::endl;
    std::cout << str(boost::format("  This parabolic curve has %d DOFs")%ndof) << std::endl;
    std::cout << str(boost::format("  t = %.15e")%duration) << std::endl;
    std::string x0VectString = GenerateStringFromVector(x0Vect);
    x0VectString = "  x0Vect = " + x0VectString;
    std::cout << x0VectString << std::endl;
    std::string x1VectString = GenerateStringFromVector(x1Vect);
    x0VectString = "  x1Vect = " + x1VectString;
    std::cout << x1VectString << std::endl;
    std::string swList = GenerateStringFromVector(switchpointsList);
    swList = "  Switch points = " + swList;
    std::cout << swList << std::endl;
}

void ParabolicCurvesND::EvalPos(dReal t, std::vector<dReal>& xVect) const {
    BOOST_ASSERT(t >= -epsilon);
    BOOST_ASSERT(t <= duration + epsilon);
    if (t <= 0) {
        xVect = x0Vect;
        return;
    }
    else if (t >= duration) {
        xVect = x1Vect;
        return;
    }

    xVect.resize(ndof);
    for (size_t i = 0; i < ndof; i++) {
        xVect[i] = curves[i].EvalPos(t);
    }
    return;
}

void ParabolicCurvesND::EvalVel(dReal t, std::vector<dReal>& vVect) const {
    BOOST_ASSERT(t >= -epsilon);
    BOOST_ASSERT(t <= duration + epsilon);
    if (t <= 0) {
        vVect = v0Vect;
        return;
    }
    else if (t >= duration) {
        vVect = v1Vect;
        return;
    }

    vVect.resize(ndof);
    for (size_t i = 0; i < ndof; i++) {
        vVect[i] = curves[i].EvalVel(t);
    }
    return;
}

void ParabolicCurvesND::EvalAcc(dReal t, std::vector<dReal>& aVect) const {
    BOOST_ASSERT(t >= -epsilon);
    BOOST_ASSERT(t <= duration + epsilon);
    if (t < 0) {
        t = 0;
    }
    else if (t > duration) {
        t = duration;
    }

    aVect.resize(ndof);
    for (size_t i = 0; i < ndof; i++) {
        aVect[i] = curves[i].EvalAcc(t);
    }
    return;
}

void ParabolicCurvesND::GetPeaks(std::vector<dReal>& bminVect, std::vector<dReal>& bmaxVect) const {
    GetPeaks(0, duration, bminVect, bmaxVect);
    return;
}

void ParabolicCurvesND::GetPeaks(dReal ta, dReal tb, std::vector<dReal>& bminVect, std::vector<dReal>& bmaxVect) const {
    bminVect.resize(ndof);
    bminVect.resize(ndof);
    for (size_t i = 0; i < ndof; ++i) {
        curves[i].GetPeaks(ta, tb, bminVect[i], bmaxVect[i]);
    }
    return;
}

void ParabolicCurvesND::Reset() {
    ndof = 0;
    duration = 0;
    x0Vect.clear();
    x1Vect.clear();
    dVect.clear();
    v1Vect.clear();
    v0Vect.clear();
    switchpointsList.clear();
    curves.clear();
    constraintCheckedVect.clear();
    constraintChecked = false;
    return;
}

void ParabolicCurvesND::SetConstant(const std::vector<dReal>& _x0Vect, dReal t) {
    if (t < 0) {
        RAMP_OPTIM_WARN("Invalid time given: t = %.15e", t);
    }
    RAMP_OPTIM_ASSERT(t >= 0);

    ndof = _x0Vect.size();
    std::vector<ParabolicCurve> _curves(ndof);
    for (size_t idof = 0; idof < ndof; ++idof) {
        ParabolicCurve curve;
        curve.SetConstant(_x0Vect[idof], t);
        _curves[idof] = curve;
    }

    Initialize(_curves);
    return;
}

void ParabolicCurvesND::SetSegment(const std::vector<dReal>& _x0Vect, const std::vector<dReal>& _x1Vect, const std::vector<dReal>& _v0Vect, const std::vector<dReal>& _v1Vect, dReal t) {
    if (t <= 0) {
        RAMP_OPTIM_WARN("Invalid time given: t = %.15e", t);
    }
    RAMP_OPTIM_ASSERT(t > 0);

    ndof = _x0Vect.size();
    std::vector<ParabolicCurve> _curves(ndof);
    for (size_t idof = 0; idof < ndof; ++idof) {
        ParabolicCurve curve;
        curve.SetSegment(_x0Vect[idof], _x1Vect[idof], _v0Vect[idof], _v1Vect[idof], t);
        _curves[idof] = curve;
    }

    Initialize(_curves);
    return;
}

void ParabolicCurvesND::SetZeroDuration(const std::vector<dReal>& _x0Vect, const std::vector<dReal>& _v0Vect) {
    ndof = _x0Vect.size();
    RAMP_OPTIM_ASSERT(_v0Vect.size() == ndof);

    std::vector<ParabolicCurve> _curves(ndof);
    for (size_t idof = 0; idof < ndof; ++idof) {
        ParabolicCurve curve;
        curve.SetZeroDuration(_x0Vect[idof], _v0Vect[idof]);
        _curves[idof] = curve;
    }

    Initialize(_curves);
    return;
}

void ParabolicCurvesND::Cut(dReal t, ParabolicCurvesND &remCurvesND) {
    BOOST_ASSERT(t >= -epsilon);
    BOOST_ASSERT(t <= duration + epsilon);
    if (t <= 0) {
        remCurvesND.Initialize(curves);
        SetZeroDuration(x0Vect, v0Vect);
        return;
    }
    else if (t >= duration) {
        remCurvesND.SetZeroDuration(x1Vect, v1Vect);
        return;
    }

    std::vector<ParabolicCurve> leftHalf = curves;
    std::vector<ParabolicCurve> rightHalf(ndof);

    for (size_t idof = 0; idof < ndof; ++idof) {
        leftHalf[idof].Cut(t, rightHalf[idof]);
    }

    Initialize(leftHalf);
    remCurvesND.Initialize(rightHalf);
    return;
}

void ParabolicCurvesND::TrimFront(dReal t) {
    BOOST_ASSERT(t >= -epsilon);
    BOOST_ASSERT(t <= duration + epsilon);
    if (t <= 0) {
        return;
    }
    else if (t >= duration) {
        SetZeroDuration(x1Vect, v1Vect);
        return;
    }

    std::vector<ParabolicCurve> newCurves = curves;

    for (size_t idof = 0; idof < ndof; ++idof) {
        newCurves[idof].TrimFront(t);
    }

    Initialize(newCurves);
    return;
}

void ParabolicCurvesND::TrimBack(dReal t) {
    BOOST_ASSERT(t >= -epsilon);
    BOOST_ASSERT(t <= duration + epsilon);
    if (t <= 0) {
        SetZeroDuration(x0Vect, v0Vect);
        return;
    }
    else if (t >= duration) {
        return;
    }

    std::vector<ParabolicCurve> newCurves = curves;

    for (size_t idof = 0; idof < ndof; ++idof) {
        newCurves[idof].TrimBack(t);
    }

    Initialize(newCurves);
    return;
}

void ParabolicCurvesND::ToString(std::string &s) const {
    s = str(boost::format("%d\n%.15e\n")%ndof%duration);
    std::string separator;
    std::string dummy;
    for (size_t idof = 0; idof < ndof; ++idof) {
        separator = "";
        for (std::vector<Ramp>::const_iterator itramp = curves[idof].ramps.begin(); itramp != curves[idof].ramps.end(); ++itramp) {
            dummy = str(boost::format("%s%.15e %.15e %.15e %.15e")%separator%(itramp->v0)%(itramp->a)%(itramp->duration)%(itramp->x0));
            s = s + dummy;
            separator = " ";
        }
        s = s + "\n";
    }
    return;
}

void ParabolicCurvesND::Serialize(std::ostream &O) const {
    O << ndof     << std::endl;
    O << duration << std::endl;

    std::string separator;
    for (size_t idof = 0; idof < ndof; ++idof) {
        separator = "";
        for (std::vector<Ramp>::const_iterator itramp = curves[idof].ramps.begin(); itramp != curves[idof].ramps.end(); ++itramp) {
            O << separator << itramp->v0;
            separator = " ";
            O << separator << itramp->a;
            O << separator << itramp->duration;
            O << separator << itramp->x0;
        }
        O << std::endl;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
std::string GenerateStringFromVector(const std::vector<dReal>& vect) {
    std::string s = "[ ";
    std::string separator = "";
    for (size_t i = 0; i < vect.size(); ++i) {
        s = s + separator + str(boost::format("%.15e")%(vect[i]));
        separator = ", ";
    }
    s = s + "]";
    return s;
}

} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
