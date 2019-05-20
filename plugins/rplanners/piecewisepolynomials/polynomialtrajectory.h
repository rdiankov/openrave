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
#ifndef PIECEWISE_POLY_TRAJECTORY_H
#define PIECEWISE_POLY_TRAJECTORY_H

#include <vector>
#include <openrave/openrave.h>

namespace OpenRAVE {

namespace PiecewisePolynomialsInternal {

class Polynomial {
public:
    /*
       \params vcoeffs vector of polynomial coefficients, starting from the weakest term. That is,
               this polynomial p is described by p(t) = c[0] + c[1]*t + c[2]*t**2 + ... + c[n]*t**n,
               where n is the degree of this polynomial
     */
    Polynomial(const std::vector<dReal>& c);
    ~Polynomial()
    {
    }

    //
    // Functions
    //
    /// \brief (Re)Initialize this polynomial with the given coefficients.
    void Initialize(const std::vector<dReal>& c);

    /// \brief Update the weakest term coefficient.
    void UpdateInitialValue(dReal c0);

    /// \brief Evaluate this polynomial at time t.
    dReal Eval(dReal t) const;

    /// \brief Evaluate the first derivative of this polynomial at time t.
    dReal Evald1(dReal t) const;

    /// \brief Evaluate the second derivative of this polynomial at time t.
    dReal Evald2(dReal t) const;

    /// \brief Evaluate the third derivative of this polynomial at time t.
    dReal Evald3(dReal t) const;

    /// \brief Evaluate the n-th derivative of this polynomial at time t.
    dReal Evaldn(dReal t, size_t n) const;

    // /// \brief Return the polynomial d/dt p(t)
    // Polynomial Differentiate();

    // /// \brief Return the polynomial int p(t) dt
    // Polynomial Integrate();

    /// \brief Return
    inline const std::vector<std::pair<dReal, dReal> >& GetExtrema() const
    {
        return vpextrema;
    }

    /// \brief Find all local extremas of this polynomial (or some n-th order derivative of this
    ///        polynomial).
    void _FindAllLocalExtrema();

    //
    // Members
    //
    size_t idegree; ///< the degree of this polynomial
    std::vector<dReal> vcoeffs; ///< vector of coefficients of this polynomial (weakest term first)
    std::vector<dReal> vcoeffsd; ///< vector of coefficients of the first derivative of this polynomial
    std::vector<dReal> vcoeffsdd; ///< vector of coefficients of the second derivative of this polynomial
    std::vector<dReal> vcoeffsddd; ///< vector of coefficients of the third derivative of this polynomial
    std::vector<std::pair<dReal, dReal> > vpextrema; ///< vector of pairs (t, p(t)) where each t is such that p(t) is a local extrema

    mutable std::vector<dReal> _vcurcoeffs;
}; // end class Polynomial


} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE
#endif
