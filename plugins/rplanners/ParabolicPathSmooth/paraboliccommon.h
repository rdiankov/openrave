// -*- coding: utf-8 -*-
#ifndef PARABOLIC_RAMP_COMMON_H
#define PARABOLIC_RAMP_COMMON_H

#include <openrave/openrave.h>

namespace ParabolicRampInternal {

typedef OpenRAVE::dReal Real;
typedef std::vector<Real> Vector;

#define PARABOLIC_RAMP_PLOG RAVELOG_VERBOSE
#define PARABOLIC_RAMP_PERROR RAVELOG_ERROR
#define PARABOLICWARN RAVELOG_WARN
#define PARABOLIC_RAMP_ASSERT BOOST_ASSERT

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
    if (x>=Inf) {
        return 1;
    }
    else if(x <=-Inf) {
        return -1;
    }
    else {
        return 0;
    }
}

inline bool IsFinite(Real x) {
    return OpenRAVE::RaveFabs(x)<Inf;
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
    return (x>0 ? 1 : (x<0 ? -1 : 0));
}
inline Real Min(Real x,Real y) {
    return (x<y ? x : y);
}
inline Real Max(Real x,Real y) {
    return (x>y ? x : y);
}
inline bool FuzzyZero(Real x,Real tol) {
    return OpenRAVE::RaveFabs(x)<=tol;
}
inline bool FuzzyEquals(Real x,Real y,Real tol) {
    return OpenRAVE::RaveFabs(x-y)<=tol;
}
inline bool FuzzyInRange(Real x, Real xmin, Real xmax, Real tol) {
    return x>=xmin-tol && x <=xmax+tol;
}
inline void Swap(Real& x,Real& y) {
    Real temp=x; x=y; y=temp;
}

//solves the quadratic formula and returns the number of roots found
inline int SolveQuadratic(Real a, Real b, Real c, Real& x1, Real& x2)
{
    return OpenRAVE::mathextra::solvequad(a, b, c, x1, x2);
}

//return a value x in [xmin,xmax] such that |a*x - b| <= epsilon*max(|a||b|)
//for ill posed problems choose x=0
bool SafeEqSolve(Real a,Real b,Real epsilon,Real xmin,Real xmax,Real& x);

bool SaveRamp(const char* fn,Real x0,Real dx0,Real x1,Real dx1, Real a,Real v,Real t);

bool LoadRamp(FILE* f,Real& x0,Real& dx0,Real& x1,Real& dx1, Real& a,Real& v,Real& t);
bool LoadRamp(const char* fn,Real& x0,Real& dx0,Real& x1,Real& dx1, Real& a,Real& v,Real& t);

}

#endif
