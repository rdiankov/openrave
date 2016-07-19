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
namespace OpenRAVE {

namespace RampOptimizerInternal {

const static dReal epsilon = 1e-12;
const static dReal inf = 1e300;

class Ramp {
public:
    Ramp () {
    }
    Ramp(dReal v0, dReal a, dReal dur, dReal x0=0);
    ~Ramp () {
    }

    // Functions
    dReal EvalPos(dReal t) const;
    dReal EvalVel(dReal t) const;
    dReal EvalAcc(dReal t) const;
    void GetPeaks(dReal& bmin, dReal& bmax) const;
    void GetPeaks(dReal ta, dReal tb, dReal& bmin, dReal& bmax) const;
    void Initialize(dReal v0, dReal a, dReal dur, dReal x0=0);
    void PrintInfo(std::string name) const;
    void PrintInfo() const {
        PrintInfo("");
    }
    void SetInitialValue(dReal newx0);
    void UpdateDuration(dReal newDuration);

    // Cut the Ramp into two halves at time t. The left half is stored in the same Ramp. The right half is
    // returned via remRamp
    void Cut(dReal t, Ramp &remRamp);
    // Cut the Ramp into two halves at time t and keep the right half.
    void TrimFront(dReal t);
    // Cut the Ramp into two halves at time t and keep the left half.
    void TrimBack(dReal t);

    // Members
    dReal v0;       // initial velocity
    dReal a;        // acceleration
    dReal duration; // total duration
    dReal x0;       // initial displacement
    dReal x1;       // final displacement
    dReal v1;       // final velocity
    dReal d;        // total displacement 'done' by this Ramp. For example, EvalPos(duration) = x0 + d = x1.
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
    dReal EvalPos(dReal t) const;
    dReal EvalVel(dReal t) const;
    dReal EvalAcc(dReal t) const;
    void FindRampIndex(dReal t, int& i, dReal& rem) const;
    void GetPeaks(dReal& bmin, dReal& bmax) const;
    void GetPeaks(dReal ta, dReal tb, dReal& bmin, dReal& bmax) const;
    void Initialize(std::vector<Ramp> ramps);
    bool IsEmpty() const {
        return ramps.size() == 0;
    }
    void PrintInfo(std::string name) const;
    void PrintInfo() const {
        PrintInfo("");
    }
    // Resize all vectors to zero and reset all dReal values to zero.
    void Reset();
    void SetConstant(dReal x0, dReal t=0);
    // Set initial value of the Curve. Also the initial value of each ramp accordingly. Note that d
    // will not be modified here.
    void SetInitialValue(dReal newx0);
    void SetSegment(dReal _x0, dReal _x1, dReal _v0, dReal _v1, dReal t);
    void SetZeroDuration(dReal _x0, dReal _v0);

    // Cut the ParabolicCurve into two halves at time t. The left half is stored in the same
    // ParabolicCurve. The right half is returned via remCurve
    void Cut(dReal t, ParabolicCurve &remCurve);
    // Cut the ParabolicCurve into two halves at time t and keep the right half.
    void TrimFront(dReal t);
    // Cut the ParabolicCurve into two halves at time t and keep the left half.
    void TrimBack(dReal t);

    // Members
    dReal x0;
    dReal x1;
    dReal duration;
    dReal d;        // total displacement 'done' by this Curve. For example, EvalPos(duration) = x0 + d = x1.
    dReal v0;
    dReal v1;
    std::vector<dReal> switchpointsList;
    std::vector<Ramp> ramps;

}; // end class ParabolicCurve

class ParabolicCurvesND {
public:
    ParabolicCurvesND() {
    }
    ParabolicCurvesND(std::vector<ParabolicCurve> curves);
    ~ParabolicCurvesND() {
    }

    // Functions
    void EvalPos(dReal t, std::vector<dReal>& xVect) const;
    void EvalVel(dReal t, std::vector<dReal>& vVect) const;
    void EvalAcc(dReal t, std::vector<dReal>& aVect) const;

    void Append(ParabolicCurvesND curvesnd);
    void GetPeaks(std::vector<dReal>& bminVect, std::vector<dReal>& bmaxVect) const;
    void GetPeaks(dReal ta, dReal tb, std::vector<dReal>& bminVect, std::vector<dReal>& bmaxVect) const;
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
    void SetInitialValues(const std::vector<dReal>& _x0Vect);
    void SetConstant(const std::vector<dReal>& _x0Vect, dReal t=0);
    void SetSegment(const std::vector<dReal>& _x0Vect, const std::vector<dReal>& _x1Vect, const std::vector<dReal>& _v0Vect, const std::vector<dReal>& _v1Vect, dReal t);
    void SetZeroDuration(const std::vector<dReal>& _x0Vect, const std::vector<dReal>& _v0Vect);

    // Cut the ParabolicCurvesND into two halves at time t. The left half is stored in the same
    // ParabolicCurvesND. The right half is returned via remCurvesND
    void Cut(dReal t, ParabolicCurvesND &remCurvesND);
    // Cut the ParabolicCurvesND into two halves at time t and keep the right half.
    void TrimFront(dReal t);
    // Cut the ParabolicCurvesND into two halves at time t and keep the left half.
    void TrimBack(dReal t);

    void ToString(std::string &s) const;
    
    size_t ndof;
    dReal duration;
    std::vector<dReal> x0Vect;
    std::vector<dReal> x1Vect;
    std::vector<dReal> dVect;
    std::vector<dReal> v0Vect;
    std::vector<dReal> v1Vect;
    std::vector<dReal> switchpointsList;
    std::vector<ParabolicCurve> curves;

    // constraintCheckedVect contains the status of whether each ndSegment (a segment between two
    // consecutive switch points) has been checked. Note that constraintCheckedVect.size() ==
    // switchpointsList.size() - 1.
    mutable std::vector<bool> constraintCheckedVect;
    mutable bool constraintChecked;
    // mutable int modified;

}; // end class ParabolicCurversND

std::string GenerateStringFromVector(const std::vector<dReal>& vect);


} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
#endif
