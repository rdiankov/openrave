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
#include "polynomialtrajectory.h"
#include <iostream>

namespace OpenRAVE {

namespace PiecewisePolynomialsInternal {

//
// Polynomial
//
Polynomial::Polynomial(const std::vector<dReal>& c)
{
    this->Initialize(c);
}

void Polynomial::Initialize(const std::vector<dReal>& c)
{
    OPENRAVE_ASSERT_OP(c.size(), >, 0);
    idegree = (c.size() - 1);
    vcoeffs = c;

    if( idegree >= 1 ) {
        vcoeffsd.resize(idegree);
        vcoeffsd.assign(vcoeffs.begin() + 1, vcoeffs.end());
        for( size_t ival = 1; ival < idegree; ++ival ) {
            vcoeffsd[ival] *= (ival + 1);
        }
    }
    else {
        vcoeffsd.resize(0);
    }
    if( idegree >= 2 ) {
        vcoeffsdd.resize(idegree - 1);
        vcoeffsdd.assign(vcoeffsd.begin() + 1, vcoeffsd.end());
        for( size_t ival = 1; ival < idegree - 1; ++ival ) {
            vcoeffsdd[ival] *= (ival + 1);
        }
    }
    else {
        vcoeffsdd.resize(0);
    }
    if( idegree >= 3 ) {
        vcoeffsddd.resize(idegree - 2);
        vcoeffsddd.assign(vcoeffsdd.begin() + 1, vcoeffsdd.end());
        for( size_t ival = 1; ival < idegree - 2; ++ival ) {
            vcoeffsddd[ival] *= (ival + 1);
        }
    }
    else {
        vcoeffsddd.resize(0);
    }

    _FindAllLocalExtrema();
}

void Polynomial::UpdateInitialValue(dReal c0)
{
    vcoeffs[0] = c0;
}

dReal Polynomial::Eval(dReal t) const
{
    dReal val = vcoeffs.back();
    for( size_t i = idegree - 1; i >= 0; --i ) {
        val = val*t + vcoeffs[i];
    }
    return val;
}

dReal Polynomial::Evald1(dReal t) const
{
    if( idegree < 1 ) {
        return 0;
    }
    dReal val = vcoeffsd.back();
    for( size_t i = idegree - 2; i >= 0; --i ) {
        val = val*t + vcoeffsd[i];
    }
    return val;
}

dReal Polynomial::Evald2(dReal t) const
{
    if( idegree < 2 ) {
        return 0;
    }
    dReal val = vcoeffsdd.back();
    for( size_t i = idegree - 3; i >= 0; --i ) {
        val = val*t + vcoeffsdd[i];
    }
    return val;
}

dReal Polynomial::Evald3(dReal t) const
{
    if( idegree < 3 ) {
        return 0;
    }
    dReal val = vcoeffsdd.back();
    for( size_t i = idegree - 4; i >= 0; --i ) {
        val = val*t + vcoeffsddd[i];
    }
    return val;
}

dReal Polynomial::Evaldn(dReal t, size_t n) const
{
    OPENRAVE_ASSERT_OP(n, >=, 0);
    switch( n ) {
    case 0:
        return Eval(t);
    case 1:
        return Evald1(t);
    case 2:
        return Evald2(t);
    case 3:
        return Evald3(t);
    }

    // The number of coefficients decreases every time we take a derivative.
    int numcoeffs = idegree + 1 - n;
    _vcurcoeffs.resize(numcoeffs);

    n = n - 3;
    _vcurcoeffs.assign(vcoeffsddd.begin() + n, vcoeffsddd.end());
    for( int i = 0; i < numcoeffs; ++i ) {
        for( int jmult = 1; jmult < n + 1; ++jmult ) {
            _vcurcoeffs[i] *= (jmult + i);
        }
    }

    dReal val = _vcurcoeffs.back();
    for( size_t i = numcoeffs - 2; i >= 0; --i ) {
        val = val*t + _vcurcoeffs[i];
    }
    return val;
}

void Polynomial::_FindAllLocalExtrema()
{
}

} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE
