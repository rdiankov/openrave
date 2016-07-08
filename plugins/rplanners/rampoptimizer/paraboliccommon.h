// -*- coding: utf-8 -*-
#ifndef RAMP_OPTIM_PARABOLIC_COMMON_H
#define RAMP_OPTIM_PARABOLIC_COMMON_H

#include <openrave/openrave.h>

namespace RampOptimizerInternal {

typedef OpenRAVE::dReal Real;
typedef std::vector<Real> Vector;

#define RAMP_OPTIM_PLOG RAVELOG_VERBOSE
#define RAMP_OPTIM_PERROR RAVELOG_ERROR
#define RAMP_OPTIM_WARN RAVELOG_WARN
#define RAMP_OPTIM_ASSERT BOOST_ASSERT

//tolerance for time
const static Real EpsilonT = 1e-8;
//tolerance for position
const static Real EpsilonX = 1e-8;
//tolerance for velocity
const static Real EpsilonV = 1e-8;
//tolerance for acceleration, should be smaller since any checks involving it do not acrue much error...
const static Real EpsilonA = 1e-9;

//can replace this with your favorite representation/tests of infinity
const static Real Inf = 1e300;

inline std::string GetDumpDirectory() {
    return OpenRAVE::RaveGetHomeDirectory();
}

inline Real Rand() {
    return OpenRAVE::RaveRandomFloat();
}

inline int IsInf(Real x) {
    if (x >= Inf) {
        return 1;
    }
    else if(x <= -Inf) {
        return -1;
    }
    else {
        return 0;
    }
}

inline bool IsFinite(Real x) {
    return OpenRAVE::RaveFabs(x) < Inf;
}

inline Real Sqr(Real x) {
    return x*x;
}
inline Real Sqrt(Real x) {
    return OpenRAVE::RaveSqrt(x);
}
inline Real Abs(Real x) {
    return OpenRAVE::RaveFabs(x);
}
inline Real Sign(Real x) {
    return (x > 0 ? 1 : (x < 0 ? -1 : 0));
}
inline Real Min(Real x,Real y) {
    return (x < y ? x : y);
}
inline Real Max(Real x,Real y) {
    return (x > y ? x : y);
}
inline bool FuzzyZero(Real x,Real tol) {
    return OpenRAVE::RaveFabs(x) <= tol;
}
inline bool FuzzyEquals(Real x,Real y,Real tol) {
    return OpenRAVE::RaveFabs(x - y) <= tol;
}
inline bool FuzzyInRange(Real x, Real xmin, Real xmax, Real tol) {
    return x >= xmin - tol && x <= xmax + tol;
}
inline void Swap(Real& x,Real& y) {
    Real temp = x; x = y; y = temp;
}

inline void LinearCombination(Real a, std::vector<Real>& v1, Real b, std::vector<Real>& v2, std::vector<Real>& v3) {
    size_t v1size = v1.size();
    RAMP_OPTIM_ASSERT(v1size == v2.size());
    if (v3.size() != v1size) {
        v3.resize(v1size);
    }
    if ((a == 1) && (b == 1)) {
        for (size_t i = 0; i < v1size; ++i) {
            v3[i] = v1[i] + v2[i];
        }
    }
    else if ((a == 1) && (b == -1)) {
        for (size_t i = 0; i < v1size; ++i) {
            v3[i] = v1[i] - v2[i];
        }
    }
    else if ((a == -1) && (b == 1)) {
        for (size_t i = 0; i < v1size; ++i) {
            v3[i] = v2[i] - v1[i];
        }
    }
    else {
        for (size_t i = 0; i < v1size; ++i) {
            v3[i] = a*v1[i] + b*v2[i];
        }
    }
}

//solves the quadratic formula and returns the number of roots found
inline int SolveQuadratic(Real a, Real b, Real c, Real& x1, Real& x2)
{
    return OpenRAVE::mathextra::solvequad(a, b, c, x1, x2);
}

//return a value x in [xmin,xmax] such that |a*x - b| <= epsilon*max(|a||b|)
//for ill posed problems choose x=0
bool SafeEqSolve(Real a, Real b, Real epsilon, Real xmin, Real xmax, Real& x);

bool SaveRamp(const char* fn, Real x0, Real dx0, Real x1, Real dx1, Real a, Real v, Real t);

bool LoadRamp(FILE* f, Real& x0, Real& dx0, Real& x1, Real& dx1, Real& a, Real& v, Real& t);
bool LoadRamp(const char* fn, Real& x0, Real& dx0, Real& x1, Real& dx1, Real& a, Real& v, Real& t);

}

#endif
