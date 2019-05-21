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
    degree = (c.size() - 1);
    vcoeffs = c;

    if( degree >= 1 ) {
        vcoeffsd.resize(degree);
        vcoeffsd.assign(vcoeffs.begin() + 1, vcoeffs.end());
        for( size_t ival = 1; ival < degree; ++ival ) {
            vcoeffsd[ival] *= (ival + 1);
        }
    }
    else {
        vcoeffsd.resize(0);
    }
    if( degree >= 2 ) {
        vcoeffsdd.resize(degree - 1);
        vcoeffsdd.assign(vcoeffsd.begin() + 1, vcoeffsd.end());
        for( size_t ival = 1; ival < degree - 1; ++ival ) {
            vcoeffsdd[ival] *= (ival + 1);
        }
    }
    else {
        vcoeffsdd.resize(0);
    }
    if( degree >= 3 ) {
        vcoeffsddd.resize(degree - 2);
        vcoeffsddd.assign(vcoeffsdd.begin() + 1, vcoeffsdd.end());
        for( size_t ival = 1; ival < degree - 2; ++ival ) {
            vcoeffsddd[ival] *= (ival + 1);
        }
    }
    else {
        vcoeffsddd.resize(0);
    }

    _FindAllLocalExtrema();
}

void Polynomial::PadCoefficients(size_t newdegree)
{
    OPENRAVE_ASSERT_OP(newdegree, >=, degree);
    vcoeffs.assign(newdegree, 0); // increase the size to newdegree and fill it with 0s.
    if( vcoeffsd.size() > 0 ) {
        vcoeffsd.assign(newdegree - 1, 0);
    }
    if( vcoeffsdd.size() > 0 ) {
        vcoeffsdd.assign(newdegree - 2, 0);
    }
    if( vcoeffsddd.size() > 0 ) {
        vcoeffsddd.assign(newdegree - 3, 0);
    }
}

void Polynomial::UpdateInitialValue(dReal c0)
{
    vcoeffs[0] = c0;
}

dReal Polynomial::Eval(dReal t) const
{
    dReal val = vcoeffs.back();
    for( size_t i = degree - 1; i >= 0; --i ) {
        val = val*t + vcoeffs[i];
    }
    return val;
}

dReal Polynomial::Evald1(dReal t) const
{
    if( degree < 1 ) {
        return 0;
    }
    dReal val = vcoeffsd.back();
    for( size_t i = degree - 2; i >= 0; --i ) {
        val = val*t + vcoeffsd[i];
    }
    return val;
}

dReal Polynomial::Evald2(dReal t) const
{
    if( degree < 2 ) {
        return 0;
    }
    dReal val = vcoeffsdd.back();
    for( size_t i = degree - 3; i >= 0; --i ) {
        val = val*t + vcoeffsdd[i];
    }
    return val;
}

dReal Polynomial::Evald3(dReal t) const
{
    if( degree < 3 ) {
        return 0;
    }
    dReal val = vcoeffsdd.back();
    for( size_t i = degree - 4; i >= 0; --i ) {
        val = val*t + vcoeffsddd[i];
    }
    return val;
}

dReal Polynomial::Evaldn(dReal t, size_t n) const
{
    OPENRAVE_ASSERT_OP(n, >=, 0);
    if( degree < n ) {
        return 0;
    }
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
    size_t numcoeffs = degree + 1 - n;
    _vcurcoeffs.resize(numcoeffs);

    n = n - 3;
    _vcurcoeffs.assign(vcoeffsddd.begin() + n, vcoeffsddd.end());
    for( size_t i = 0; i < numcoeffs; ++i ) {
        for( size_t jmult = 1; jmult < n + 1; ++jmult ) {
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

void Polynomial::Serialize(std::ostream& O) const
{
    O << vcoeffs[0];
    for( size_t i = 1; i < vcoeffs.size(); ++i ) {
        O << " " << vcoeffs[i];
    }
}

//
// Chunk
//
Chunk::Chunk(const dReal duration, const std::vector<Polynomial> vpolynomials)
{
    this->duration = duration;
    this->vpolynomials = vpolynomials;
    this->dof = this->vpolynomials.size();
    this->degree = vpolynomials.front().degree;
    for( size_t i = 1; i < vpolynomials.size(); ++i ) {
        if( vpolynomials[i].degree > this->degree ) {
            this->degree = vpolynomials[i].degree;
        }
    }
    for( size_t i = 1; i < this->vpolynomials.size(); ++i ) {
        if( this->vpolynomials[i].degree < this->degree ) {
            this->vpolynomials[i].PadCoefficients(this->degree);
        }
    }
}

void Chunk::UpdateInitialValues(std::vector<dReal>&vinitialvalues)
{
    OPENRAVE_ASSERT_OP(vinitialvalues.size(), ==, dof);
    for( size_t idof = 0; idof < dof; ++idof ) {
        vpolynomials[idof].UpdateInitialValue(vinitialvalues[idof]);
    }
}

void Chunk::Eval(dReal t, std::vector<dReal>& res) const
{
    res.resize(dof);
    for( size_t idof = 0; idof < dof; ++idof ) {
        res[idof] = vpolynomials[idof].Eval(t);
    }
}

void Chunk::Evald1(dReal t, std::vector<dReal>& res) const
{
    res.resize(dof);
    for( size_t idof = 0; idof < dof; ++idof ) {
        res[idof] = vpolynomials[idof].Evald1(t);
    }
}

void Chunk::Evald2(dReal t, std::vector<dReal>& res) const
{
    res.resize(dof);
    for( size_t idof = 0; idof < dof; ++idof ) {
        res[idof] = vpolynomials[idof].Evald2(t);
    }
}

void Chunk::Evald3(dReal t, std::vector<dReal>& res) const
{
    res.resize(dof);
    for( size_t idof = 0; idof < dof; ++idof ) {
        res[idof] = vpolynomials[idof].Evald3(t);
    }
}

void Chunk::Evaldn(dReal t, size_t n, std::vector<dReal>& res) const
{
    res.resize(dof);
    for( size_t idof = 0; idof < dof; ++idof ) {
        res[idof] = vpolynomials[idof].Evaldn(t, n);
    }
}

void Chunk::Serialize(std::ostream& O) const
{
    O << duration;
    O << "\n";
    O << dof;
    for( size_t ipoly = 0; ipoly < vpolynomials.size(); ++ipoly ) {
        O << "\n";
        vpolynomials[ipoly].Serialize(O);
    }
}

//
// Piecewise Polynomial Trajectory
//
PiecewisePolynomialTrajectory::PiecewisePolynomialTrajectory(const std::vector<Chunk>& vchunks)
{
    this->vchunks = vchunks;
    _UpdateChunksVector();

    this->degree = this->vchunks.front().degree;
    this->dof = this->vchunks.front().dof;

    vswitchtimes.reserve(this->vchunks.size() + 1);
    vswitchtimes.resize(0);
    dReal t = 0;
    vswitchtimes.push_back(0);
    for( size_t ichunk = 0; ichunk < this->vchunks.size(); ++ichunk ) {
        t += this->vchunks[ichunk].duration;
        vswitchtimes.push_back(t);
    }
    this->duration = t;
}

void PiecewisePolynomialTrajectory::_UpdateChunksVector()
{
    if( vchunks.size() == 0 ) {
        return;
    }
    std::vector<dReal> vvalues;
    vchunks.front().Eval(vchunks.front().duration, vvalues);
    for( size_t ichunk = 1; ichunk < vchunks.size(); ++ichunk ) {
        OPENRAVE_ASSERT_OP(vchunks[ichunk].dof, ==, vvalues.size());
        vchunks[ichunk].UpdateInitialValues(vvalues);
        vchunks[ichunk].Eval(vchunks[ichunk].duration, vvalues);
    }
}

void PiecewisePolynomialTrajectory::FindChunkIndex(dReal t, size_t& index, dReal& remainder) const
{
    if( FuzzyZero(t, g_fEpsilon) ) {
        index = 0;
        remainder = 0;
        return;
    }
    std::vector<dReal>::const_iterator it = std::lower_bound(vswitchtimes.begin(), vswitchtimes.end(), t) - 1;
    index = it - vswitchtimes.begin();
    remainder = t - vswitchtimes[index];
    return;
}

void PiecewisePolynomialTrajectory::Eval(dReal t, std::vector<dReal>& res) const
{
    size_t index;
    dReal remainder;
    FindChunkIndex(t, index, remainder);
    vchunks[index].Eval(remainder, res);
}

void PiecewisePolynomialTrajectory::Evald1(dReal t, std::vector<dReal>& res) const
{
    size_t index;
    dReal remainder;
    FindChunkIndex(t, index, remainder);
    vchunks[index].Evald1(remainder, res);
}

void PiecewisePolynomialTrajectory::Evald2(dReal t, std::vector<dReal>& res) const
{
    size_t index;
    dReal remainder;
    FindChunkIndex(t, index, remainder);
    vchunks[index].Evald2(remainder, res);
}

void PiecewisePolynomialTrajectory::Evald3(dReal t, std::vector<dReal>& res) const
{
    size_t index;
    dReal remainder;
    FindChunkIndex(t, index, remainder);
    vchunks[index].Evald3(remainder, res);
}

void PiecewisePolynomialTrajectory::Evaldn(dReal t, size_t n, std::vector<dReal>& res) const
{
    size_t index;
    dReal remainder;
    FindChunkIndex(t, index, remainder);
    vchunks[index].Evaldn(remainder, n, res);
}

void PiecewisePolynomialTrajectory::Serialize(std::ostream& O) const
{
    vchunks.front().Serialize(O);
    for( size_t ichunk = 1; ichunk < vchunks.size(); ++ichunk ) {
        O << "\n";
        vchunks[ichunk].Serialize(O);
    }
}

} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE
