#ifndef PARABOLIC_RAMP_MATH_H
#define PARABOLIC_RAMP_MATH_H

#include <openrave/openrave.h>

namespace ParabolicRamp {

typedef OpenRAVE::dReal Real;
typedef std::vector<Real> Vector;

#define PARABOLICLOG RAVELOG_VERBOSE
#define PARABOLICWARN RAVELOG_DEBUG
#define PARABOLIC_ASSERT BOOST_ASSERT

//can replace this with your favorite representation/tests of infinity
const static Real Inf = 1e300;
inline bool IsInf(Real x) {
    return x==Inf;
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
inline void Swap(Real& x,Real& y) {
    Real temp=x; x=y; y=temp;
}

}

#endif
