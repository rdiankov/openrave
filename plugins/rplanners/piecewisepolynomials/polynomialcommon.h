// -*- coding: utf-8 -*-
// Copyright (C) 2019 Puttichai Lertkultanon
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
#ifndef PIECEWISE_POLY_POLY_COMMON_H
#define PIECEWISE_POLY_POLY_COMMON_H

#include <openrave/openrave.h>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/bindings/traits/ublas_matrix.hpp>
#include <boost/numeric/bindings/lapack/gesdd.hpp>

namespace OpenRAVE {

namespace PiecewisePolynomialsInternal {

const static dReal g_fPolynomialEpsilon = 1e-10; // tolerance for polynomial interpolation & checking operation
const static int g_nPrec = 12;         // precision when writing trajectories to files
const static dReal g_fPolynomialInf = 1e300;     // threshold for deciding if a number is infinite

inline int IsInf(dReal x)
{
    if (x >= g_fPolynomialInf) {
        return 1;
    }
    else if(x <= -g_fPolynomialInf) {
        return -1;
    }
    else {
        return 0;
    }
}

inline bool IsFinite(dReal x)
{
    return OpenRAVE::RaveFabs(x) < g_fPolynomialInf;
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

// void polyrealroots(const std::vector<dReal>& vcoeffs, std::vector<dReal>& vroots) {
//     // initialization
//     const uint32_t len = vcoeffs.size();
//     if (len < 2) {
//         vroots.clear();
//         return;
//     }
//     const uint32_t degree = len - 1;

//     // construct the companion matrix
//     using namespace boost::numeric::ublas;
//     boost::numeric::ublas::matrix<dReal, column_major> C(degree, degree); // companion matrix
//     const dReal fileadcoeff = 1.0/vcoeffs.back();
//     for( size_t i = 0; i < degree - 1; ++i ) {
//         C(i + 1, i) = 1;
//         C(i, degree - 1) = -vcoeffs[i] * fileadcoeff;
//     }
//     C(degree - 1, degree - 1) = -vcoeffs[degree - 1] * fileadcoeff;

//     boost::numeric::ublas::vector<dReal> S(degree);
//     boost::numeric::ublas::matrix<dReal, column_major> U(degree, degree), V(degree, degree);
//     int ret = boost::numeric::bindings::lapack::gesdd('O', 'A', C, S, U, V);

//     // assign to output
//     vroots.resize(0);
//     vroots.reserve(degree);
//     for (uint32_t i = 0; i < degree; ++i) {
//         std::complex<dReal> r = S(i);
//         if( FuzzyZero(r.imag(), g_fPolynomialEpsilon) ) {
//             vroots.push_back(r.real());
//         }
//     }
//     return;
// }

// Modified from mathextra.h
inline void polyroots(const int degree, const dReal* rawcoeffs, dReal* rawroots, int& numroots)
{
    using std::complex;
    BOOST_ASSERT(rawcoeffs[0] != 0);
    const dReal tol = 128.0*std::numeric_limits<dReal>::epsilon();
    const dReal tolsqrt = sqrt(std::numeric_limits<dReal>::epsilon());
    complex<dReal> coeffs[degree];
    const int maxsteps = 110;
    for(int i = 0; i < degree; ++i) {
        coeffs[i] = complex<dReal>(rawcoeffs[i+1]/rawcoeffs[0]);
    }
    complex<dReal> roots[degree];
    dReal err[degree];
    roots[0] = complex<dReal>(1,0);
    roots[1] = complex<dReal>(0.4,0.9); // any complex number not a root of unity works
    err[0] = 1.0;
    err[1] = 1.0;
    for(int i = 2; i < degree; ++i) {
        roots[i] = roots[i-1]*roots[1];
        err[i] = 1.0;
    }
    for(int step = 0; step < maxsteps; ++step) {
        bool changed = false;
        for(int i = 0; i < degree; ++i) {
            if ( err[i] >= tol ) {
                changed = true;
                // evaluate
                complex<dReal> x = roots[i] + coeffs[0];
                for(int j = 1; j < degree; ++j) {
                    x = roots[i] * x + coeffs[j];
                }
                for(int j = 0; j < degree; ++j) {
                    if( i != j ) {
                        if( roots[i] != roots[j] ) {
                            x /= (roots[i] - roots[j]);
                        }
                    }
                }
                roots[i] -= x;
                err[i] = abs(x);
            }
        }
        if( !changed ) {
            break;
        }
    }

    numroots = 0;
    bool visited[degree] = {false};
    for(int i = 0; i < degree; ++i) {
        if( !visited[i] ) {
            // might be a multiple root, in which case it will have more error than the other roots
            // find any neighboring roots, and take the average
            complex<dReal> newroot=roots[i];
            int n = 1;
            for(int j = i+1; j < degree; ++j) {
                if( abs(roots[i]-roots[j]) < 8*tolsqrt ) {
                    newroot += roots[j];
                    n += 1;
                    visited[j] = true;
                }
            }
            if( n > 1 ) {
                newroot /= n;
            }
            // there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
	    // std::cout << "root " << i << ": val=" << real(newroot) << " + " << imag(newroot) << "j\n";
            if( RaveFabs(imag(newroot)) < tolsqrt ) {
                rawroots[numroots++] = real(newroot);
            }
        }
    }
}

} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE

#endif
