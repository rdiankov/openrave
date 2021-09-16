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
    vcoeffs = c;
    Initialize();
}

void Polynomial::Initialize()
{
    degree = (vcoeffs.size() - 1);

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
    degree = newdegree;
}

void Polynomial::UpdateInitialValue(dReal c0)
{
    vcoeffs[0] = c0;
}

dReal Polynomial::Eval(dReal t) const
{
    dReal val = vcoeffs.back();
    for( int i = (int)degree - 1; i >= 0; --i ) {
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
    for( int i = (int)degree - 2; i >= 0; --i ) {
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
    for( int i = (int)degree - 3; i >= 0; --i ) {
        val = val*t + vcoeffsdd[i];
    }
    return val;
}

dReal Polynomial::Evald3(dReal t) const
{
    if( degree < 3 ) {
        return 0;
    }
    dReal val = vcoeffsddd.back();
    for( int i = (int)degree - 4; i >= 0; --i ) {
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
    for( int i = (int)numcoeffs - 2; i >= 0; --i ) {
        val = val*t + _vcurcoeffs[i];
    }
    return val;
}

void Polynomial::_FindAllLocalExtrema()
{
    if( vcoeffsd.size() == 0 ) {
        // No extrema since the function is constant
        return;
    }

    // Solve for the roots of the polynomial described by vcoeffsd to determine the points at which
    // the first derivative vanishes.

    // rawcoeffs for polyroots: strongest term first
    std::vector<dReal> rawcoeffs(degree), rawroots(degree - 1);
    int iNonZeroLeadCoeff = -1;
    for( size_t icoeff = 0; icoeff < degree; ++icoeff ) {
        rawcoeffs[icoeff] = vcoeffsd[degree - 1 - icoeff];
        if( iNonZeroLeadCoeff < 0 && rawcoeffs[icoeff] != 0 ) {
            iNonZeroLeadCoeff = icoeff;
        }
    }
    if( iNonZeroLeadCoeff < 0 || iNonZeroLeadCoeff == (int)degree - 1 ) {
        // No extrema in this case.
        return;
    }
    int numroots = 0;
    polyroots((int)degree - 1 - iNonZeroLeadCoeff, &rawcoeffs[iNonZeroLeadCoeff], &rawroots[0], numroots);
    rawroots.resize(numroots);
    vcextrema.resize(0);

    if( numroots == 0 ) {
        return;
    }

    // Collect all *critical* points in vcextrema.
    vcextrema.reserve(numroots);
    for( int iroot = 0; iroot < numroots; ++iroot ) {
        Coordinate c(rawroots[iroot], Eval(rawroots[iroot]));
        std::vector<Coordinate>::const_iterator it = std::lower_bound(vcextrema.begin(), vcextrema.end(), c);
        if( !FuzzyEquals(it->point, c.point, g_fPolynomialEpsilon) ) {
            // Insert this point only if not already in the list
            vcextrema.insert(it, c);
        }
    }

    // Determine if a critical point is a local extrema or not.
    dReal prevpoint = vcextrema[0].point - 1;
    int writeindex = 0;
    for( int readindex = 0; readindex < numroots; ++readindex ) {
        dReal leftpoint, rightpoint; // points at which to evaluate the polynomial values
        dReal leftvalue, rightvalue; // polynomial values evaluated at leftpoint and rightpoint, respectively

        leftpoint = 0.5*(prevpoint + vcextrema[readindex].point);
        if( readindex == numroots - 1 ) {
            rightpoint = vcextrema[readindex].point + 1;
        }
        else {
            rightpoint = 0.5*(vcextrema[readindex].point + vcextrema[readindex + 1].point);
        }
        leftvalue = Eval(leftpoint);
        rightvalue = Eval(rightpoint);

        prevpoint = vcextrema[readindex].point; // update prevpoint first

        if( (vcextrema[readindex].value - leftvalue) * (rightvalue - vcextrema[readindex].value) < 0 ) {
            // This point is a local extrema so keep it.
            if( readindex > writeindex ) {
                vcextrema[writeindex] = vcextrema[readindex];
            }
            ++writeindex;
        }
    }
    vcextrema.resize(writeindex);
    return;
}

void Polynomial::FindAllLocalExtrema(size_t ideriv, std::vector<Coordinate>& vcoords) const
{
    vcoords.resize(0);
    if( ideriv >= degree - 1 ) {
        // No local extrema.
        return;
    }

    // The number of coefficients after taking (ideriv + 1)-th derivative is (degree + 1) - (ideriv + 1)
    size_t numcoeffs = degree - ideriv;
    _vcurcoeffs.resize(numcoeffs); // storing strongest coeff first
    if( ideriv == 0 ) {
        for( size_t icoeff = 0; icoeff < numcoeffs; ++icoeff ) {
            _vcurcoeffs[icoeff] = vcoeffsd[degree - 1 - icoeff];
        }
    }
    else if( ideriv == 1 ) {
        for( size_t icoeff = 0; icoeff < numcoeffs; ++icoeff ) {
            _vcurcoeffs[icoeff] = vcoeffsdd[degree - 2 - icoeff];
        }
    }
    else if( ideriv == 2 ) {
        for( size_t icoeff = 0; icoeff < numcoeffs; ++icoeff ) {
            _vcurcoeffs[icoeff] = vcoeffsddd[degree - 3 - icoeff];
        }
    }
    else {
        // The remaining number of times we need to take derivatives considering that we start with
        // the coeffcients of the third derivative, which we already computed.
        size_t inewderiv = ideriv - 2;

        for( size_t icoeff = 0; icoeff < numcoeffs; ++icoeff ) {
            _vcurcoeffs[icoeff] = vcoeffsddd[degree - 3 - icoeff];
            for( size_t jmult = 1; jmult < inewderiv + 1; ++jmult ) {
                _vcurcoeffs[numcoeffs - 1 - icoeff] *= (jmult + icoeff);
            }
        }
    }

    if( 0 ) {
        std::cout << "ideriv=" << ideriv << "; (strongest term first) vcoeffs=[" << _vcurcoeffs[0];
        for( size_t icoeff = 1; icoeff < numcoeffs; ++icoeff ) {
            std::cout << ", " << _vcurcoeffs[icoeff];
        }
        std::cout << "]\n";
    }

    int numroots = 0;
    int iNonZeroLeadCoeff = 0;
    for( size_t icoeff = 0; icoeff < numcoeffs; ++icoeff ) {
        if( _vcurcoeffs[icoeff] != 0 ) {
            iNonZeroLeadCoeff = icoeff;
            break;
        }
    }
    if( iNonZeroLeadCoeff < 0 || iNonZeroLeadCoeff == (int)numcoeffs - 1 ) {
        // No extrema in this case.
        return;
    }
    std::vector<dReal> rawroots(numcoeffs - 1);
    polyroots((int)numcoeffs - 1 - iNonZeroLeadCoeff, &_vcurcoeffs[iNonZeroLeadCoeff], &rawroots[0], numroots);
    rawroots.resize(numroots);
    if( numroots == 0 ) {
        return;
    }

    // Collect all *critical* points in vcoords.
    vcoords.reserve(numroots);
    for( int iroot = 0; iroot < numroots; ++iroot ) {
        Coordinate c(rawroots[iroot], Evaldn(rawroots[iroot], ideriv));
        // std::cout << "    examining (" << c.point << ", " << c.value << ")\n";
        if( vcoords.size() == 0 ) {
            vcoords.push_back(c);
        }
        else {
            std::vector<Coordinate>::const_iterator it = std::lower_bound(vcoords.begin(), vcoords.end(), c);
            if( it != vcoords.end() ) {
                if( !FuzzyEquals(it->point, c.point, g_fPolynomialEpsilon) ) {
                    // Insert this point only if not already in the list
                    vcoords.insert(it, c);
                }
            }
        }
    }

    // Determine if a critical point is a local extrema or not.
    dReal prevpoint = vcoords[0].point - 1;
    int writeindex = 0;
    for( int readindex = 0; readindex < (int)vcoords.size(); ++readindex ) {
        dReal leftpoint, rightpoint; // points at which to evaluate the polynomial values
        dReal leftvalue, rightvalue; // polynomial values evaluated at leftpoint and rightpoint, respectively

        leftpoint = 0.5*(prevpoint + vcoords[readindex].point);
        if( readindex == numroots - 1 ) {
            rightpoint = vcoords[readindex].point + 1;
        }
        else {
            rightpoint = 0.5*(vcoords[readindex].point + vcoords[readindex + 1].point);
        }
        leftvalue = Evaldn(leftpoint, ideriv);
        rightvalue = Evaldn(rightpoint, ideriv);

        prevpoint = vcoords[readindex].point; // update prevpoint first

        if( (vcoords[readindex].value - leftvalue) * (rightvalue - vcoords[readindex].value) < 0 ) {
            // This point is a local extrema so keep it.
            if( readindex > writeindex ) {
                vcoords[writeindex] = vcoords[readindex];
            }
            ++writeindex;
        }
    }
    vcoords.resize(writeindex);
    return;
}

void Polynomial::Serialize(std::ostream& O) const
{
    O << vcoeffs.size();
    for( size_t i = 0; i < vcoeffs.size(); ++i ) {
        O << " " << vcoeffs[i];
    }
}

void Polynomial::Deserialize(std::istream& I)
{
    size_t numCoeffs;
    I >> numCoeffs;
    vcoeffs.resize(numCoeffs);
    for( size_t i = 0; i < numCoeffs; ++i ) {
        I >> vcoeffs[i];
    }
    Initialize();
}

//
// Chunk
//
Chunk::Chunk(const dReal duration, const std::vector<Polynomial>& vpolynomials)
{
    Initialize(duration, vpolynomials);
}

void Chunk::UpdateInitialValues(std::vector<dReal>&vinitialvalues)
{
    OPENRAVE_ASSERT_OP(vinitialvalues.size(), ==, dof);
    for( size_t idof = 0; idof < dof; ++idof ) {
        vpolynomials[idof].UpdateInitialValue(vinitialvalues[idof]);
    }
}

void Chunk::Initialize(const dReal duration, const std::vector<Polynomial>& vpolynomials)
{
    this->duration = duration;
    this->vpolynomials = vpolynomials;
    Initialize();
}

void Chunk::Initialize()
{
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

void Chunk::Cut(dReal t, Chunk& remChunk)
{
    // TODO: Need to find a better way to not have to create these vectors every time this function is called.
    std::vector<Polynomial> vpoly(this->dof);
    std::vector<dReal> vcoeffs(this->degree + 1);
    dReal fMult;
    for( size_t idof = 0; idof < this->dof; ++idof ) {
        fMult = 1.0;
        for( size_t icoeff = 0; icoeff < this->degree + 1; ++icoeff ) {
            vcoeffs[icoeff] = this->vpolynomials[idof].Evaldn(t, icoeff)/fMult;
            fMult *= (icoeff + 1);
        }
        vpoly[idof].Initialize(vcoeffs);
    }
    remChunk.Initialize(this->duration - t, vpoly);

    this->duration = t;
    remChunk.constraintChecked = this->constraintChecked;
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

void Chunk::Deserialize(std::istream& I)
{
    I >> duration;
    I >> dof;
    vpolynomials.resize(dof);
    for( size_t ipoly = 0; ipoly < dof; ++ipoly ) {
        vpolynomials[ipoly].Deserialize(I);
    }
    Initialize();
}

void Chunk::SetConstant(const std::vector<dReal>& x0Vect, const dReal duration, const size_t degree)
{
    this->dof = x0Vect.size();
    this->degree = degree;
    this->duration = duration;
    vpolynomials.resize(this->dof);
    std::vector<dReal> vcoeffs(this->degree + 1, 0);
    for( size_t idof = 0; idof < this->dof; ++idof ) {
        // All other coefficients are zero.
        vcoeffs[0] = x0Vect[idof];
        vpolynomials[idof].Initialize(vcoeffs);
    }
}

//
// Piecewise Polynomial Trajectory
//
PiecewisePolynomialTrajectory::PiecewisePolynomialTrajectory(const std::vector<Chunk>& vchunks)
{
    Initialize(vchunks);
}

void PiecewisePolynomialTrajectory::Initialize(const std::vector<Chunk>& vchunks)
{
    this->vchunks = vchunks;
    Initialize();
}

void PiecewisePolynomialTrajectory::Initialize()
{
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
    // TODO: probably make it consider epsilon
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
    O << vchunks.size();
    for( size_t ichunk = 0; ichunk < vchunks.size(); ++ichunk ) {
        O << "\n";
        vchunks[ichunk].Serialize(O);
    }
}

void PiecewisePolynomialTrajectory::Deserialize(std::istream& I)
{
    size_t numChunks;
    I >> numChunks;
    vchunks.resize(numChunks);
    for( size_t ichunk = 0; ichunk < numChunks; ++ichunk ) {
        vchunks[ichunk].Deserialize(I);
    }

    Initialize();
}

void PiecewisePolynomialTrajectory::ReplaceSegment(dReal t0, dReal t1, const std::vector<Chunk>& vchunks)
{
    OPENRAVE_ASSERT_OP(t0, <=, t1);
    size_t index0, index1;
    dReal rem0, rem1;
    this->FindChunkIndex(t0, index0, rem0);
    this->FindChunkIndex(t1, index1, rem1);
    if( index0 == index1 ) {
        Chunk tempChunk;
        this->vchunks[index0].Cut(rem1, tempChunk); // now tempChunk stores the portion from rem1 to duration
        this->vchunks.insert(this->vchunks.begin() + index0 + 1, tempChunk); // tempChunk is copied into the traj
        this->vchunks[index0].Cut(rem0, tempChunk); // now chunk index0 stores the portion from 0 to rem0
        this->vchunks.insert(this->vchunks.begin() + index0 + 1, vchunks.begin(), vchunks.end()); // insert vchunks after index0
    }
    else {
        Chunk tempChunk;
        // Manage chunk index1
        this->vchunks[index1].Cut(rem1, tempChunk);
        this->vchunks[index1] = tempChunk; // replace chunk index1 by the portion from rem1 to duration

        // Manager chunk index0
        this->vchunks[index0].Cut(rem0, tempChunk); // TODO: maybe define TrimBack function is better

        // Insert the input chunks before index1
        this->vchunks.insert(this->vchunks.begin() + index1, vchunks.begin(), vchunks.end());

        // Remove the previous segment from index0 to index1
        this->vchunks.erase(this->vchunks.begin() + index0 + 1, this->vchunks.begin() + index1);
    }
    this->Initialize();
}

} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE
