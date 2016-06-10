// -*- coding: utf-8 -*-
#include "ramp.h"
#include <iostream>

// Lines with ///< are those likely to introduce numerical errors. Will fix that later.

namespace ParabolicRampInternal {

////////////////////////////////////////////////////////////////////////////////
// Ramp
Ramp::Ramp(Real v0_, Real a_, Real dur_, Real x0_)  {
    assert(dur_ > -epsilon);

    v0 = v0_;
    a = a_;
    duration = dur_;
    x0 = x0_;

    v1 = v0 + (a*duration);
    d = duration*(v0 + 0.5*a*duration);
}

Real Ramp::EvalPos(Real t) const {
    std::cout << str(boost::format("t = %.15e")%t) << std::endl;
    assert(t > -epsilon);
    assert(t < duration + epsilon);
    if (t < 0) t = 0;
    else if (t > duration) t = duration;

    return t*(v0 + 0.5*a*t) + x0;
}

Real Ramp::EvalVel(Real t) const {
    assert(t > -epsilon);
    assert(t < duration + epsilon);
    if (t < 0) t = 0;
    else if (t > duration) t = duration;

    return v0 + (a*t);
}

Real Ramp::EvalAcc(Real t) const {
    assert(t > -epsilon);
    assert(t < duration + epsilon);
    return a;
}

void Ramp::Initialize(Real v0_, Real a_, Real dur_, Real x0_) {
    assert(dur_ > -epsilon);

    v0 = v0_;
    a = a_;
    duration = dur_;
    x0 = x0_;

    v1 = v0 + (a*duration);
    d = duration*(v0 + 0.5*a*duration);
}

void Ramp::PrintInfo(std::string name) const {
    std::cout << "Ramp information: " << name << std::endl;
    std::cout << str(boost::format("  v0 = %.15e")%v0) << std::endl;
    std::cout << str(boost::format("   a = %.15e")%a) << std::endl;
    std::cout << str(boost::format("   t = %.15e")%duration) << std::endl;
    std::cout << str(boost::format("  x0 = %.15e")%x0) << std::endl;
    std::cout << str(boost::format("  v1 = %.15e")%v1) << std::endl;
    std::cout << str(boost::format("   d = %.15e")%d) << std::endl;
}

void Ramp::UpdateDuration(Real newDuration) {
    assert(newDuration > -epsilon);
    if (newDuration < 0) newDuration = 0;

    duration = newDuration;
    v1 = v0 + (a*duration);
    d = duration*(v0 + 0.5*a*duration);
}

////////////////////////////////////////////////////////////////////////////////
// ParabolicCurve
ParabolicCurve::ParabolicCurve(std::vector<Ramp> rampsIn) {
    assert(!rampsIn.empty());

    ramps.reserve(rampsIn.size());
    switchpointsList.reserve(rampsIn.size() + 1);
    d = 0.0;
    duration = 0.0;
    switchpointsList.push_back(duration);
    x0 = rampsIn[0].x0;

    for (std::size_t i = 0; i != rampsIn.size(); ++i) {
        ramps.push_back(rampsIn[i]);
        ramps[i].x0 = d;
        d = d + ramps[i].d; ///<
        duration = duration + ramps[i].duration; ///<
        switchpointsList.push_back(duration);
    }
}

void ParabolicCurve::Append(ParabolicCurve curve) {
    assert(!curve.IsEmpty());

    std::size_t prevLength = 0;
    std::size_t sz = curve.ramps.size();
    if (ramps.size() == 0) {
        switchpointsList.reserve(sz + 1);
        d = 0.0;
        duration = 0.0;
        switchpointsList.push_back(duration);
        x0 = curve.ramps[0].x0;
    }
    prevLength = ramps.size();
    ramps.reserve(prevLength + sz);

    for (std::size_t i = 0; i != sz; ++i) {
        ramps.push_back(curve.ramps[i]);
        ramps.back().x0 = d;
        d = d + ramps.back().d; ///<
        duration = duration + ramps.back().duration; ///<
        switchpointsList.push_back(duration);
    }
}

void ParabolicCurve::FindRampIndex(Real t, int& index, Real& remainder) const {
    assert(t > -epsilon);
    assert(t < duration + epsilon);
    if (t < epsilon) {
        index = 0;
        remainder = 0;
    }
    else if (t > duration - epsilon) {
        index = ((int) ramps.size()) - 1;
        remainder = ramps.back().duration;
    }
    else {
        index = 0;
        // Iterate through switchpointsList
        std::vector<Real>::const_iterator it = switchpointsList.begin();
        while (it != switchpointsList.end() && t < *it) {
            index++;
            it++;
        }
        assert(index < (int)switchpointsList.size() - 1);
        remainder = t - *it;
    }
    return;
}

void ParabolicCurve::Initialize(std::vector<Ramp> rampsIn) {
    assert(!rampsIn.empty());
    assert(ramps.empty()); // can be initialized only once

    ramps.reserve(rampsIn.size());
    switchpointsList.reserve(rampsIn.size() + 1);
    d = 0.0;
    duration = 0.0;
    switchpointsList.push_back(duration);
    x0 = rampsIn[0].x0;

    for (std::size_t i = 0; i != rampsIn.size(); ++i) {
        ramps.push_back(rampsIn[i]);
        ramps[i].x0 = d;
        d = d + ramps[i].d; ///<
        duration = duration + ramps[i].duration; ///<
        switchpointsList.push_back(duration);
    }
}

void ParabolicCurve::PrintInfo(std::string name) const {
    std::cout << "ParabolicCurve information: " << name << std::endl;
    std::cout << str(boost::format("  This parabolic curve consists of %d ramps")%(ramps.size())) << std::endl;
    std::cout << str(boost::format("  v0 = %.15e")%(ramps[0].v0)) << std::endl;
    std::cout << str(boost::format("   t = %.15e")%duration) << std::endl;
    std::cout << str(boost::format("  x0 = %.15e")%x0) << std::endl;
    std::cout << str(boost::format("   d = %.15e")%d) << std::endl;
    std::string swList = GenerateStringFromVector(switchpointsList);
    swList = "  Switch points = " + swList;
    std::cout << swList << std::endl;
}

Real ParabolicCurve::EvalPos(Real t) const {
    assert(t > -epsilon);
    assert(t < duration + epsilon);
    if (t < 0) t = 0;
    else if (t > duration) t = duration;

    int index;
    Real remainder;
    FindRampIndex(t, index, remainder);
    return ramps[index].EvalPos(remainder);
}

Real ParabolicCurve::EvalVel(Real t) const {
    assert(t > -epsilon);
    assert(t < duration + epsilon);
    if (t < 0) t = 0;
    else if (t > duration) t = duration;

    int index;
    Real remainder;
    FindRampIndex(t, index, remainder);
    return ramps[index].EvalVel(remainder);
}

Real ParabolicCurve::EvalAcc(Real t) const {
    assert(t > -epsilon);
    assert(t < duration + epsilon);
    if (t < 0) t = 0;
    else if (t > duration) t = duration;

    int index;
    Real remainder;
    FindRampIndex(t, index, remainder);
    return ramps[index].a;
}

////////////////////////////////////////////////////////////////////////////////
// ParabolicCurvesND
ParabolicCurvesND::ParabolicCurvesND(std::vector<ParabolicCurve> curvesIn) {
    assert(!curvesIn.empty());

    ndof = curvesIn.size();
    // Here we need to check whether every curve has (roughly) the same duration
    ///////////////////
    Real minDur = curvesIn[0].duration;
    for (std::vector<ParabolicCurve>::const_iterator it = curvesIn.begin() + 1; it != curvesIn.end(); ++it) {
        assert(FuzzyEquals(it->duration, minDur, epsilon));
        minDur = Min(minDur, it->duration);
    }
    // Need to think about how to handle discrepancies in durations.

    curves = curvesIn;
    duration = minDur;

    x0Vect.reserve(ndof);
    for (int i = 0; i < ndof; ++i) {
        x0Vect.push_back(curves[i].x0);
    }

    // Manage switch points (later I will merge this for loop with the one above)
    switchpointsList = curves[0].switchpointsList;
    std::vector<Real>::iterator it;
    for (int i = 0; i < ndof; i++) {
        for (std::size_t j = 1; j != curves[i].switchpointsList.size() - 1; ++j) {
            // Iterate from the second element to the second last element (the first and the last
            // switch points should be the same for every DOF)
            it = std::lower_bound(switchpointsList.begin(), switchpointsList.end(), curves[i].switchpointsList[j]);
            if (!FuzzyEquals(curves[i].switchpointsList[j], *it, epsilon)) {
                // Insert only non-redundant switch points
                switchpointsList.insert(it, curves[i].switchpointsList[j]);
            }
        }
    }
}

void ParabolicCurvesND::Append(ParabolicCurvesND curvesnd) {
    assert(!curvesnd.IsEmpty());

    if (curves.empty()) {
        curves = curvesnd.curves;
        ndof = curves.size();
        duration = curvesnd.duration;
        x0Vect = curvesnd.x0Vect;
        switchpointsList = curvesnd.switchpointsList;
    }
    else {
        assert(curvesnd.ndof == ndof);
        Real originalDur = duration;
        duration = duration + curvesnd.duration; ///<
        for (int i = 0; i < ndof; i++) {
            curves[i].Append(curvesnd.curves[i]);
        }
        std::vector<Real> tempSwitchpointsList = curvesnd.switchpointsList;
        // Add every element in tempSwitchpointsList by originalDur
        for (std::vector<Real>::iterator it = tempSwitchpointsList.begin(); it != tempSwitchpointsList.end(); ++it) {
            *it += originalDur; ///<
        }
        switchpointsList.reserve(switchpointsList.size() + tempSwitchpointsList.size());
        switchpointsList.insert(switchpointsList.end(), tempSwitchpointsList.begin(), tempSwitchpointsList.end());
    }
    return;
}

void ParabolicCurvesND::Initialize(std::vector<ParabolicCurve> curvesIn) {
    assert(!curvesIn.empty());

    ndof = curvesIn.size();
    // Here we need to check whether every curve has (roughly) the same duration
    ///////////////////
    Real minDur = curvesIn[0].duration;
    for (std::vector<ParabolicCurve>::const_iterator it = curvesIn.begin() + 1; it != curvesIn.end(); ++it) {
        assert(FuzzyEquals(it->duration, minDur, epsilon));
        minDur = Min(minDur, it->duration);
    }
    // Need to think about how to handle discrepancies in durations.

    curves = curvesIn;
    duration = minDur;

    x0Vect.reserve(ndof);
    for (int i = 0; i < ndof; ++i) {
        x0Vect.push_back(curves[i].x0);
    }

    // Manage switch points (later I will merge this for loop with the one above)
    switchpointsList = curves[0].switchpointsList;
    std::vector<Real>::iterator it;
    for (int i = 0; i < ndof; i++) {
        for (std::size_t j = 1; j != curves[i].switchpointsList.size() - 1; ++j) {
            // Iterate from the second element to the second last element (the first and the last
            // switch points should be the same for every DOF)
            it = std::lower_bound(switchpointsList.begin(), switchpointsList.end(), curves[i].switchpointsList[j]);
            if (!FuzzyEquals(curves[i].switchpointsList[j], *it, epsilon)) {
                // Insert only non-redundant switch points
                switchpointsList.insert(it, curves[i].switchpointsList[j]);
            }
        }
    }
}

void ParabolicCurvesND::PrintInfo(std::string name) const {
    std::cout << "ParabolicCurvesND information: " << name << std::endl;
    std::cout << str(boost::format("  This parabolic curve has %d DOFs")%ndof) << std::endl;
    std::cout << str(boost::format("  t = %.15e")%duration) << std::endl;
    std::string x0VectString = GenerateStringFromVector(x0Vect);
    x0VectString = "  x0Vect = " + x0VectString;
    std::cout << x0VectString << std::endl;
    std::string swList = GenerateStringFromVector(switchpointsList);
    swList = "  Switch points = " + swList;
    std::cout << swList << std::endl;
}

std::vector<Real> ParabolicCurvesND::EvalPos(Real t) const {
    assert(t > -epsilon);
    assert(t < duration + epsilon);
    if (t < 0) t = 0;
    else if (t > duration) t = duration;

    std::vector<Real> xVect(ndof);
    for (int i = 0; i < ndof; i++) {
        xVect[i] = curves[i].EvalPos(t);
    }
    return xVect;
}

std::vector<Real> ParabolicCurvesND::EvalVel(Real t) const {
    assert(t > -epsilon);
    assert(t < duration + epsilon);
    if (t < 0) t = 0;
    else if (t > duration) t = duration;

    std::vector<Real> vVect(ndof);
    for (int i = 0; i < ndof; i++) {
        vVect[i] = curves[i].EvalVel(t);
    }
    return vVect;
}

std::vector<Real> ParabolicCurvesND::EvalAcc(Real t) const {
    assert(t > -epsilon);
    assert(t < duration + epsilon);
    if (t < 0) t = 0;
    else if (t > duration) t = duration;

    std::vector<Real> aVect(ndof);
    for (int i = 0; i < ndof; i++) {
        aVect[i] = curves[i].EvalAcc(t);
    }
    return aVect;
}

std::string GenerateStringFromVector(const std::vector<Real>& vect) {
    std::string s = "[ ";
    std::string separator = "";
    for (size_t i = 0; i < vect.size(); ++i) {
        s = s + separator + str(boost::format("%.15e")%(vect[i]));
        separator = ", ";
    }
    s = s + "]";
    return s;
}

} // end namespace ParabolicRampInternal
