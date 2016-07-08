// -*- coding: utf-8 -*-
// Copyright (C) 2016 Puttichai Lertkultanon <L.Puttichai@gmail.com>
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
#include "paraboliccommon.h"

// extern "C" {
// int fast_expansion_sum_zeroelim(int elen, double* e, int flen, double* f, double* h);
// }

namespace RampOptimizerInternal {

const static Real epsilon = 1e-10;//1e-8;
const static Real inf = 1e300;

class Ramp {
public:
    Ramp () {
    }
    Ramp(Real v0, Real a, Real dur, Real x0=0);
    ~Ramp () {
    }

    // Functions
    Real EvalPos(Real t) const;
    Real EvalVel(Real t) const;
    Real EvalAcc(Real t) const;
    void GetPeaks(Real& bmin, Real& bmax) const;
    void Initialize(Real v0, Real a, Real dur, Real x0=0);
    void PrintInfo(std::string name) const;
    void PrintInfo() const {
        PrintInfo("");
    }
    void SetInitialValue(Real newx0);
    void UpdateDuration(Real newDuration);

    // Members
    Real v0;       // initial velocity
    Real a;        // acceleration
    Real duration; // total duration
    Real x0;       // initial displacement
    Real x1;       // final displacement
    Real v1;       // final velocity
    Real d;        // total displacement 'done' by this Ramp. For example, EvalPos(duration) = x0 + d = x1.
}; // end class Ramp

class ParabolicCurve {
public:
    ParabolicCurve() {
    }
    ParabolicCurve(std::vector<Ramp> ramps);
    ~ParabolicCurve() {
    }

    // Functions
    void Append(ParabolicCurve curve);
    Real EvalPos(Real t) const;
    Real EvalVel(Real t) const;
    Real EvalAcc(Real t) const;
    void FindRampIndex(Real t, int& i, Real& rem) const;
    void GetPeaks(Real& bmin, Real& bmax) const;
    void Initialize(std::vector<Ramp> ramps);
    bool IsEmpty() const {
        return ramps.size() == 0;
    }
    void PrintInfo(std::string name) const;
    void PrintInfo() const {
        PrintInfo("");
    }
    // Resize all vectors to zero and reset all Real values to zero.
    void Reset();
    // Set initial value of the Curve. Also the initial value of each ramp accordingly. Note that d
    // will not be modified here.
    void SetInitialValue(Real newx0);

    // Members
    Real x0;
    Real x1;
    Real duration;
    Real d;        // total displacement 'done' by this Curve. For example, EvalPos(duration) = x0 + d = x1.
    Real v0;
    Real v1;
    std::vector<Real> switchpointsList;
    std::vector<Ramp> ramps;

}; // end class ParabolicCurve

class ParabolicCurvesND {
public:
    ParabolicCurvesND() : constraintchecked(0), modified(0) {
    }
    ParabolicCurvesND(std::vector<ParabolicCurve> curves);
    ~ParabolicCurvesND() {
    }

    // Functions
    void EvalPos(Real t, std::vector<Real>& xVect) const;
    void EvalVel(Real t, std::vector<Real>& vVect) const;
    void EvalAcc(Real t, std::vector<Real>& aVect) const;

    void Append(ParabolicCurvesND curvesnd);
    void GetPeaks(std::vector<Real>& bminVect, std::vector<Real>& bmaxVect) const;
    void Initialize(std::vector<ParabolicCurve> curves);
    bool IsEmpty() const {
        return curves.size() == 0;
    }
    void PrintInfo(std::string name) const;
    void PrintInfo() const {
        PrintInfo("");
    }
    // Resize all vectors to zero.
    void Reset();
    
    int ndof;
    Real duration;
    std::vector<Real> x0Vect;
    std::vector<Real> x1Vect;
    std::vector<Real> dVect;
    std::vector<Real> v0Vect;
    std::vector<Real> v1Vect;
    std::vector<Real> switchpointsList;
    std::vector<ParabolicCurve> curves;

    mutable int constraintchecked;
    mutable int modified;

}; // end class ParabolicCurversND

std::string GenerateStringFromVector(const std::vector<Real>& vect);


} // end namespace RampOptimizerInternal
#endif
