// -*- coding: utf-8 -*-
#ifndef RAMP_OPTIM_PARABOLIC_COMMON_H
#define RAMP_OPTIM_PARABOLIC_COMMON_H

#include <openrave/openrave.h>

namespace OpenRAVE {

namespace RampOptimizerInternal {

#define RAMP_OPTIM_PLOG RAVELOG_VERBOSE
#define RAMP_OPTIM_PERROR RAVELOG_ERROR
#define RAMP_OPTIM_WARN RAVELOG_WARN
#define RAMP_OPTIM_ASSERT BOOST_ASSERT

//tolerance for time
const static dReal EpsilonT = 1e-8;
//tolerance for position
const static dReal EpsilonX = 1e-8;
//tolerance for velocity
const static dReal EpsilonV = 1e-8;
//tolerance for acceleration, should be smaller since any checks involving it do not acrue much error...
const static dReal EpsilonA = 1e-9;

//can replace this with your favorite representation/tests of infinity
const static dReal Inf = 1e300;

inline std::string GetDumpDirectory() {
    return RaveGetHomeDirectory();
}

inline dReal Rand() {
    return RaveRandomFloat();
}

inline int IsInf(dReal x) {
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

inline bool IsFinite(dReal x) {
    return OpenRAVE::RaveFabs(x) < Inf;
}

inline dReal Sqr(dReal x) {
    return x*x;
}
inline dReal Sqrt(dReal x) {
    return OpenRAVE::RaveSqrt(x);
}
inline dReal Abs(dReal x) {
    return OpenRAVE::RaveFabs(x);
}
inline dReal Sign(dReal x) {
    return (x > 0 ? 1 : (x < 0 ? -1 : 0));
}
inline dReal Min(dReal x,dReal y) {
    return (x < y ? x : y);
}
inline dReal Max(dReal x,dReal y) {
    return (x > y ? x : y);
}
inline bool FuzzyZero(dReal x,dReal tol) {
    return OpenRAVE::RaveFabs(x) <= tol;
}
inline bool FuzzyEquals(dReal x,dReal y,dReal tol) {
    return OpenRAVE::RaveFabs(x - y) <= tol;
}
inline bool FuzzyInRange(dReal x, dReal xmin, dReal xmax, dReal tol) {
    return x >= xmin - tol && x <= xmax + tol;
}
inline void Swap(dReal& x,dReal& y) {
    dReal temp = x; x = y; y = temp;
}

inline void LinearCombination(dReal a, std::vector<dReal>& v1, dReal b, std::vector<dReal>& v2, std::vector<dReal>& v3) {
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
inline int SolveQuadratic(dReal a, dReal b, dReal c, dReal& x1, dReal& x2)
{
    return mathextra::solvequad(a, b, c, x1, x2);
}

//return a value x in [xmin,xmax] such that |a*x - b| <= epsilon*max(|a||b|)
//for ill posed problems choose x=0
bool SafeEqSolve(dReal a, dReal b, dReal epsilon, dReal xmin, dReal xmax, dReal& x);

} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
#endif
