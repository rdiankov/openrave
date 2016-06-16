//-*- coding: utf-8 -*-
#ifndef PARABOLIC_RAMP_H
#define PARABOLIC_RAMP_H

#include <vector>
#include <cassert>
#include <openrave/openrave.h>
#include <boost/format.hpp>
#include "paraboliccommon.h"

// extern "C" {
// int fast_expansion_sum_zeroelim(int elen, double* e, int flen, double* f, double* h);
// }

namespace RampOptimizerInternal {

// #define OpenRAVE::dReal Real
// typedef double Real;
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

    void Initialize(Real v0, Real a, Real dur, Real x0=0);
    void PrintInfo(std::string name) const;
    void PrintInfo() const {
        PrintInfo("");
    }
    void UpdateDuration(Real newDuration);

    // Members
    Real v0;       // initial velocity
    Real a;        // acceleration
    Real duration; // total duration
    Real x0;       // initial displacement

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
    Real EvalPos(Real t) const;
    Real EvalVel(Real t) const;
    Real EvalAcc(Real t) const;

    void Append(ParabolicCurve curve);
    void FindRampIndex(Real t, int& i, Real& rem) const;
    void Initialize(std::vector<Ramp> ramps);
    bool IsEmpty() const {
        return ramps.size() == 0;
    }
    void PrintInfo(std::string name) const;
    void PrintInfo() const {
        PrintInfo("");
    }
    // Set initial value of the Curve. Also the initial value of each ramp accordingly. Note that d
    // will not be modified here.
    void SetInitialValue(Real newx0);

    // Members
    Real x0;
    Real duration;
    Real d;        // total displacement 'done' by this Curve. For example, EvalPos(duration) = x0 + d = x1.
    Real v0;
    Real v1;
    std::vector<Real> switchpointsList;
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
    std::vector<Real> EvalPos(Real t) const;
    std::vector<Real> EvalVel(Real t) const;
    std::vector<Real> EvalAcc(Real t) const;

    void Append(ParabolicCurvesND curvesnd);
    void Initialize(std::vector<ParabolicCurve> curves);
    bool IsEmpty() const {
        return curves.size() == 0;
    }
    void PrintInfo(std::string name) const;
    void PrintInfo() const {
        PrintInfo("");
    }

    int ndof;
    Real duration;
    std::vector<Real> x0Vect;
    std::vector<Real> dVect;
    std::vector<Real> v0Vect;
    std::vector<Real> v1Vect;
    std::vector<Real> switchpointsList;
    std::vector<ParabolicCurve> curves;

}; // end class ParabolicCurversND

std::string GenerateStringFromVector(const std::vector<Real>& vect);


} // end namespace RampOptimizerInternal
#endif
