// -*- coding: utf-8 -*-
#ifndef RAMP_OPTIM_PARABOLIC_COMMON_H
#define RAMP_OPTIM_PARABOLIC_COMMON_H

#include <openrave/openrave.h>

namespace OpenRAVE {

namespace RampOptimizerInternal {

const static dReal g_fRampEpsilon = 1e-10; // tolerance for ramp computation & checking operation
const static int g_nPrec = 12;             // precision when writing trajectories to files
const static dReal g_fRampInf = 1e300;     // threshold for deciding if a number is infinite

inline std::string GetDumpDirectory()
{
    return RaveGetHomeDirectory();
}

inline dReal Rand()
{
    return RaveRandomFloat();
}

inline int IsInf(dReal x)
{
    if (x >= g_fRampInf) {
        return 1;
    }
    else if(x <= -g_fRampInf) {
        return -1;
    }
    else {
        return 0;
    }
}

inline bool IsFinite(dReal x)
{
    return OpenRAVE::RaveFabs(x) < g_fRampInf;
}

inline dReal Sqr(dReal x)
{
    return x*x;
}

inline dReal Sqrt(dReal x)
{
    return OpenRAVE::RaveSqrt(x);
}

inline dReal Abs(dReal x)
{
    return OpenRAVE::RaveFabs(x);
}

inline dReal Sign(dReal x)
{
    return (x > 0 ? 1 : (x < 0 ? -1 : 0));
}

inline dReal Min(dReal x,dReal y)
{
    return (x < y ? x : y);
}

inline dReal Max(dReal x,dReal y)
{
    return (x > y ? x : y);
}

inline bool FuzzyZero(dReal x,dReal tol)
{
    return OpenRAVE::RaveFabs(x) <= tol;
}

inline bool FuzzyEquals(dReal x,dReal y,dReal tol)
{
    return OpenRAVE::RaveFabs(x - y) <= tol;
}

inline bool FuzzyInRange(dReal x, dReal xmin, dReal xmax, dReal tol)
{
    return x >= xmin - tol && x <= xmax + tol;
}

inline void Swap(dReal& x,dReal& y)
{
    dReal temp = x; x = y; y = temp;
}

inline void LinearCombination(dReal a, const std::vector<dReal>& v1, dReal b, const std::vector<dReal>& v2, std::vector<dReal>& res)
{
    // Users need to make sure all input vector dimensions are compatible
    if (res.size() != v1.size()) {
        res.resize(v1.size());
    }
    for (size_t i = 0; i < v1.size(); ++i) {
        res[i] = a*v1[i] + b*v2[i];
    }
}

/// \brief Compute res = v1 + v2
inline void AddVector(const std::vector<dReal>& v1, const std::vector<dReal>& v2, std::vector<dReal>& res)
{
    // Users need to make sure all input vector dimensions are compatible
    if (res.size() != v1.size()) {
        res.resize(v1.size());
    }
    for (size_t i = 0; i < v1.size(); ++i) {
        res[i] = v1[i] + v2[i];
    }
}

/// \brief Compute v1 += v2
inline void AddVector2(std::vector<dReal>& v1, const std::vector<dReal>& v2)
{
    for (size_t i = 0; i < v1.size(); ++i) {
        v1[i] += v2[i];
    }
}

/// \brief Compute v1 += a*v2
inline void AddVector3(std::vector<dReal>& v1, dReal a, const std::vector<dReal>& v2)
{
    for (size_t i = 0; i < v1.size(); ++i) {
        v1[i] += a*v2[i];
    }
}
    
/// \brief Compute res = v1 - v2
inline void SubtractVector(const std::vector<dReal>& v1, const std::vector<dReal>& v2, std::vector<dReal>& res)
{
    // Users need to make sure all input vector dimensions are compatible
    if (res.size() != v1.size()) {
        res.resize(v1.size());
    }
    for (size_t i = 0; i < v1.size(); ++i) {
        res[i] = v1[i] - v2[i];
    }
}

/// \brief Compute v1 -= v2
inline void SubtractVector2(std::vector<dReal>& v1, const std::vector<dReal>& v2)
{
    for (size_t i = 0; i < v1.size(); ++i) {
        v1[i] -= v2[i];
    }
}

/// \brief Compute v1 -= a*v2
inline void SubTractVector3(std::vector<dReal>& v1, dReal a, const std::vector<dReal>& v2)
{
    for (size_t i = 0; i < v1.size(); ++i) {
        v1[i] -= a*v2[i];
    }
}

/// \brief void Multiply the vector by the given constant
inline void ScaleVector(std::vector<dReal>& v, dReal scale)
{
    for (size_t i = 0; i < v.size(); ++i) {
        v[i] *= scale;
    }
}

/// \brief Solve the quadratic equation and return the number of roots found
inline int SolveQuadratic(dReal a, dReal b, dReal c, dReal& x1, dReal& x2)
{
    return mathextra::solvequad(a, b, c, x1, x2);
}

/// \brief Return a value x in [xmin,xmax] such that |a*x - b| <= epsilon*max(|a||b|) for ill posed
/// problems choose x = 0
bool SafeEqSolve(dReal a, dReal b, dReal epsilon, dReal xmin, dReal xmax, dReal& x);

} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
#endif
