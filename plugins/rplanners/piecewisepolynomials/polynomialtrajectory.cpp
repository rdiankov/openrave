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
Polynomial::Polynomial(const dReal T, const std::vector<dReal>& c)
{
    this->Initialize(T, c);
}

void Polynomial::Initialize(const dReal T, const std::vector<dReal>& c)
{
    OPENRAVE_ASSERT_OP(c.size(), >, 0);
    vcoeffs = c;
    duration = T;
    Initialize();
}

void Polynomial::Initialize()
{
    degree = (vcoeffs.size() - 1);

    // Invalidate derivative coefficients
    _bInitCoeffs1 = false;
    _bInitCoeffs2 = false;
    _bInitCoeffs3 = false;

    displacement = Eval(duration) - Eval(0);
    _bExtremaComputed = false; // need to invalidate vcextrema
}

void Polynomial::_EnsureInitCoeffs1() const
{
    if( _bInitCoeffs1 ) {
        return;
    }
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
    _bInitCoeffs1 = true;
}

void Polynomial::_EnsureInitCoeffs2() const
{
    if( _bInitCoeffs2 ) {
        return;
    }
    _EnsureInitCoeffs1();
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
    _bInitCoeffs2 = true;
}

void Polynomial::_EnsureInitCoeffs3() const
{
    if( _bInitCoeffs3 ) {
        return;
    }
    _EnsureInitCoeffs2();
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
    _bInitCoeffs3 = true;
}

void Polynomial::PadCoefficients(const size_t newdegree)
{
    OPENRAVE_ASSERT_OP(newdegree, >=, degree);
    const size_t numcoeffs = newdegree + 1;
    vcoeffs.resize(numcoeffs, 0); // increase the size to numcoeffs and fill it with 0s.
    if( _bInitCoeffs1 && vcoeffsd.size() > 0 ) {
        vcoeffsd.resize(numcoeffs - 1, 0);
    }
    if( _bInitCoeffs2 && vcoeffsdd.size() > 0 ) {
        vcoeffsdd.assign(numcoeffs - 2, 0);
    }
    if( _bInitCoeffs3 && vcoeffsddd.size() > 0 ) {
        vcoeffsddd.assign(numcoeffs - 3, 0);
    }
    degree = newdegree;
}

void Polynomial::UpdateInitialValue(const dReal c0)
{
    const dReal translationOffset = c0 - vcoeffs[0];
    // Update the initial value
    vcoeffs[0] = c0;

    if( _bExtremaComputed ) {
        // If extrema are already computed, can adjust them directly
        for( std::vector<Coordinate>::iterator itcoord = vcextrema.begin(); itcoord != vcextrema.end(); ++itcoord ) {
            itcoord->value += translationOffset;
        }
    }
}

void Polynomial::UpdateDuration(const dReal T)
{
    duration = T;
    displacement = Eval(duration) - Eval(0);
}

void Polynomial::Reparameterize(const dReal t0)
{
    // Perform change of variable: u = t - t0
    // Compute coefficients of the new polynomial by doing Taylor series expansion at t = t0.
    dReal fMult = 1.0;
    for( size_t icoeff = 0; icoeff < this->degree; ++icoeff ) {
        vcoeffs[icoeff] = this->Evaldn(t0, icoeff)/fMult;
        fMult *= (icoeff + 1);
    }
    this->Initialize();
}

dReal Polynomial::Eval(dReal t) const
{
    if( t <= 0 ) {
        return vcoeffs[0];
    }
    else if( t > duration ) {
        t = duration;
    }
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
    _EnsureInitCoeffs1();
    if( t <= 0 ) {
        return vcoeffsd[0];
    }
    else if( t > duration ) {
        t = duration;
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
    _EnsureInitCoeffs2();
    if( t <= 0 ) {
        return vcoeffsdd[0];
    }
    else if( t > duration ) {
        t = duration;
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
    _EnsureInitCoeffs3();
    if( t <= 0 ) {
        return vcoeffsddd[0];
    }
    else if( t > duration ) {
        t = duration;
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

    if( t < 0 ) {
        t = 0;
    }
    else if( t > duration ) {
        t = duration;
    }
    // The number of coefficients decreases every time we take a derivative.
    size_t numcoeffs = degree + 1 - n;
    _vcurcoeffs.resize(numcoeffs);

    n = n - 3;
    _EnsureInitCoeffs3();
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

Polynomial Polynomial::Differentiate(const size_t ideriv) const
{
    if( ideriv == 0 ) {
        return Polynomial(duration, vcoeffs); // return a copy of itself
    }
    else if( ideriv == 1 ) {
        _EnsureInitCoeffs1();
        return Polynomial(duration, vcoeffsd);
    }
    else if( ideriv == 2 ) {
        _EnsureInitCoeffs2();
        return Polynomial(duration, vcoeffsdd);
    }
    else if( ideriv == 3 ) {
        _EnsureInitCoeffs3();
        return Polynomial(duration, vcoeffsddd);
    }
    else if( ideriv <= degree ) {
        _EnsureInitCoeffs3();
        size_t inewderiv = ideriv - 3;
        _vcurcoeffs.resize(vcoeffsddd.size() - inewderiv);
        _vcurcoeffs.assign(vcoeffsddd.begin() + inewderiv, vcoeffsddd.end());
        for( size_t icoeff = 0; icoeff < _vcurcoeffs.size(); ++icoeff ) {
            for( size_t mult = 1; mult <= inewderiv; ++mult ) {
                _vcurcoeffs[icoeff] *= (mult + icoeff);
            }
        }
        return Polynomial(duration, _vcurcoeffs);
    }
    else {
        return Polynomial(duration, {0});
    }
}

dReal Polynomial::Evali1(dReal t, const dReal c) const
{
    if( t < 0 ) {
        t = 0;
    }
    if( t > duration ) {
        t = duration;
    }

    if( degree == 0 ) {
        return t*vcoeffs[0] + c;
    }

    dReal val = (vcoeffs[degree]/(degree + 1)) * t;
    for( int i = (int)degree - 1; i >= 0; --i ) {
        val = (val + vcoeffs[i]/(i + 1)) * t;
    }
    return val + c;
}

Polynomial Polynomial::Integrate(const dReal c) const
{
    // Reusing _vcurcoeffs as a temporary container
    _vcurcoeffs.resize(vcoeffs.size() + 1);
    _vcurcoeffs[0] = c;
    _vcurcoeffs[1] = vcoeffs[0];
    for( int div = 2; div < (int)_vcurcoeffs.size(); ++div ) {
        _vcurcoeffs[div] = vcoeffs[div - 1]/div;
    }
    return Polynomial(duration, _vcurcoeffs);
}

Polynomial& Polynomial::operator=(const Polynomial& r)
{
    if( this == &r ) {
        return *this;
    }

    degree = r.degree;
    vcoeffs = r.vcoeffs;
    _bInitCoeffs1 = r._bInitCoeffs1;
    if( r._bInitCoeffs1 ) {
        vcoeffsd = r.vcoeffsd;
    }
    _bInitCoeffs2 = r._bInitCoeffs2;
    if( r._bInitCoeffs2 ) {
        vcoeffsdd = r.vcoeffsdd;
    }
    _bInitCoeffs3 = r._bInitCoeffs3;
    if( r._bInitCoeffs3 ) {
        vcoeffsddd = r.vcoeffsddd;
    }
    displacement = r.displacement;
    duration = r.duration;
    _bExtremaComputed = r._bExtremaComputed;
    if( r._bExtremaComputed ) {
        vcextrema = r.vcextrema;
    }
    // Do not copy _vcurcoeffs since it is only used as cache.
    return *this;
}

void Polynomial::_FindAllLocalExtrema() const
{
    _bExtremaComputed = true;
    vcextrema.resize(0);
    _EnsureInitCoeffs1();
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
    if( degree - 1 - iNonZeroLeadCoeff == 2 ) {
        // Use a closed-form formula for quadratic equations to speed up.
        dReal a = rawcoeffs[iNonZeroLeadCoeff], b = rawcoeffs[iNonZeroLeadCoeff + 1], c = rawcoeffs[iNonZeroLeadCoeff + 2];
        dReal det = b*b - 4*a*c;
        const dReal tol = 64.0*std::numeric_limits<dReal>::epsilon();
        if( det >= -tol ) {
            if( det <= tol ) {
                numroots = 1;
                rawroots[0] = -0.5*b/a;
            }
            else {
                numroots = 2;
                dReal temp;
                if( b >= 0 ) {
                    temp = -0.5*(b + RaveSqrt(det));
                }
                else {
                    temp = -0.5*(b - RaveSqrt(det));
                }
                rawroots[0] = temp/a;
                rawroots[1] = c/temp;
            }
        }
        rawroots.resize(numroots);
    }
    else {
        polyroots((int)degree - 1 - iNonZeroLeadCoeff, &rawcoeffs[iNonZeroLeadCoeff], &rawroots[0], numroots);
        rawroots.resize(numroots);
    }

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
    int numDistinctRoots = (int)vcextrema.size();
    int writeindex = 0;
    for( int readindex = 0; readindex < numDistinctRoots; ++readindex ) {
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
    if( ideriv == 0 ) {
        vcoords = GetExtrema();
        return;
    }
    Polynomial newpoly = this->Differentiate(ideriv);
    vcoords = newpoly.GetExtrema();
    return;
}

void Polynomial::Serialize(std::ostream& O) const
{
    O << duration;
    O << " " << vcoeffs.size();
    for( size_t i = 0; i < vcoeffs.size(); ++i ) {
        O << " " << vcoeffs[i];
    }
}

void Polynomial::Deserialize(std::istream& I)
{
    size_t numCoeffs;
    I >> duration;
    I >> numCoeffs;
    vcoeffs.resize(numCoeffs);
    for( size_t i = 0; i < numCoeffs; ++i ) {
        I >> vcoeffs[i];
    }
    Initialize();
}

//
// PiecewisePolynomial
//
PiecewisePolynomial::PiecewisePolynomial(std::vector<Polynomial>& polynomialsIn)
{
    Initialize(polynomialsIn);
}

void PiecewisePolynomial::Initialize(std::vector<Polynomial>& polynomialsIn)
{
    BOOST_ASSERT(!polynomialsIn.empty());
    // This will invalidate polynomialsIn
    _vpolynomials.swap(polynomialsIn);
    dReal duration = 0;
    for( std::vector<Polynomial>::const_iterator itpoly = _vpolynomials.begin(); itpoly != _vpolynomials.end(); ++itpoly ) {
        duration += itpoly->duration;
    }
    _duration = duration;

    UpdateInitialValue(_vpolynomials[0].vcoeffs[0]);
}

void PiecewisePolynomial::Initialize(Polynomial& polynomialIn)
{
    _vpolynomials.resize(0);
    _vpolynomials.reserve(1);
    _vpolynomials.emplace_back(polynomialIn);

    dReal duration = 0;
    for( std::vector<Polynomial>::const_iterator itpoly = _vpolynomials.begin(); itpoly != _vpolynomials.end(); ++itpoly ) {
        duration += itpoly->duration;
    }
    _duration = duration;

    UpdateInitialValue(_vpolynomials[0].vcoeffs[0]);
}

void PiecewisePolynomial::Append(Polynomial& newPolynomial)
{
    _vpolynomials.emplace_back(newPolynomial);
    dReal duration = 0;
    for( std::vector<Polynomial>::const_iterator itpoly = _vpolynomials.begin(); itpoly != _vpolynomials.end(); ++itpoly ) {
        duration += itpoly->duration;
    }
    _duration = duration;

    UpdateInitialValue(_vpolynomials[0].vcoeffs[0]);
}

void PiecewisePolynomial::UpdateInitialValue(const dReal c0)
{
    dReal newFirstCoeff = c0;
    for( std::vector<Polynomial>::iterator itpoly = _vpolynomials.begin(); itpoly != _vpolynomials.end(); ++itpoly ) {
        itpoly->UpdateInitialValue(newFirstCoeff);
        newFirstCoeff += itpoly->displacement;
    }
}

void PiecewisePolynomial::FindPolynomialIndex(const dReal t, size_t& index, dReal& remainder) const
{
    if( t <= g_fEpsilonForTimeInstant ) {
        index = 0;
        remainder = 0;
        return;
    }
    else if( t >= _duration - g_fEpsilonForTimeInstant ) {
        index = _vpolynomials.size() - 1;
        remainder = _vpolynomials.back().duration;
        return;
    }
    else {
        // Convention: if t lies exactly at a junction between _vpolynomials[i]
        // and _vpolynomials[i + 1], we return index = i + 1 and remainder = 0
        size_t testIndex = 0;
        dReal currentTime = 0;
        std::vector<Polynomial>::const_iterator itpoly = _vpolynomials.begin();
        while( itpoly != _vpolynomials.end() && t >= currentTime - g_fEpsilonForTimeInstant ) {
            currentTime += itpoly->duration;
            itpoly++;
            testIndex++;
        }
        index = testIndex - 1;
        remainder = t - (currentTime - (itpoly - 1)->duration);
        if( FuzzyZero(remainder, g_fEpsilonForTimeInstant) ) {
            remainder = 0;
        }
        return;
    }
}

dReal PiecewisePolynomial::Eval(dReal t) const
{
    if( t <= g_fEpsilonForTimeInstant ) {
        return _vpolynomials.front().Eval(0);
    }
    else if( t >= _duration - g_fEpsilonForTimeInstant ) {
        return _vpolynomials.back().Eval(_vpolynomials.back().duration);
    }

    size_t index = 0;
    dReal remainder = 0;
    FindPolynomialIndex(t, index, remainder);
    return _vpolynomials[index].Eval(remainder);
}

dReal PiecewisePolynomial::Evald1(dReal t) const
{
    if( t <= g_fEpsilonForTimeInstant ) {
        return _vpolynomials.front().Evald1(0);
    }
    else if( t >= _duration - g_fEpsilonForTimeInstant ) {
        return _vpolynomials.back().Evald1(_vpolynomials.back().duration);
    }

    size_t index = 0;
    dReal remainder = 0;
    FindPolynomialIndex(t, index, remainder);
    return _vpolynomials[index].Evald1(remainder);
}

dReal PiecewisePolynomial::Evald2(dReal t) const
{
    if( t <= g_fEpsilonForTimeInstant ) {
        return _vpolynomials.front().Evald2(0);
    }
    else if( t >= _duration - g_fEpsilonForTimeInstant ) {
        return _vpolynomials.back().Evald2(_vpolynomials.back().duration);
    }

    size_t index = 0;
    dReal remainder = 0;
    FindPolynomialIndex(t, index, remainder);
    return _vpolynomials[index].Evald2(remainder);
}

dReal PiecewisePolynomial::Evald3(dReal t) const
{
    if( t <= g_fEpsilonForTimeInstant ) {
        return _vpolynomials.front().Evald3(0);
    }
    else if( t >= _duration - g_fEpsilonForTimeInstant ) {
        return _vpolynomials.back().Evald3(_vpolynomials.back().duration);
    }

    size_t index = 0;
    dReal remainder = 0;
    FindPolynomialIndex(t, index, remainder);
    return _vpolynomials[index].Evald3(remainder);
}

dReal PiecewisePolynomial::Evaldn(dReal t, size_t n) const
{
    if( t <= g_fEpsilonForTimeInstant ) {
        return _vpolynomials.front().Evaldn(0, n);
    }
    else if( t >= _duration - g_fEpsilonForTimeInstant ) {
        return _vpolynomials.back().Evaldn(_vpolynomials.back().duration, n);
    }

    size_t index = 0;
    dReal remainder = 0;
    FindPolynomialIndex(t, index, remainder);
    return _vpolynomials[index].Evaldn(remainder, n);
}

dReal PiecewisePolynomial::Evali1(dReal t, const dReal c) const
{
    if( t <= g_fEpsilonForTimeInstant ) {
        return _vpolynomials.front().Evali1(0, c);
    }

    if( t >= _duration - g_fEpsilonForTimeInstant ) {
        t = _duration;
    }

    size_t index = 0;
    dReal remainder = 0;
    FindPolynomialIndex(t, index, remainder);

    dReal newInitialValue = c;
    for( size_t ipoly = 0; ipoly < index; ++ipoly ) {
        newInitialValue = _vpolynomials[ipoly].Evali1(_vpolynomials[ipoly].duration, newInitialValue);
    }
    return _vpolynomials[index].Evali1(remainder, newInitialValue);
}

void PiecewisePolynomial::Serialize(std::ostream& O) const {
    O << _vpolynomials.size();
    for( size_t ipoly = 0; ipoly < _vpolynomials.size(); ++ipoly ) {
        O << "\n";
        _vpolynomials[ipoly].Serialize(O);
    }
}

void PiecewisePolynomial::Deserialize(std::istream& I) {
    size_t numPolynomials = 0;
    I >> numPolynomials;
    std::vector<Polynomial> vPolynomialsIn(numPolynomials);
    for( size_t ipoly = 0; ipoly < numPolynomials; ++ipoly ) {
        vPolynomialsIn[ipoly].Deserialize(I);
    }
    Initialize(vPolynomialsIn);
}

PiecewisePolynomial PiecewisePolynomial::Differentiate(const size_t ideriv) const
{
    std::vector<Polynomial> vNewPolynomials;
    vNewPolynomials.reserve(_vpolynomials.size());
    for( std::vector<Polynomial>::const_iterator itpoly = _vpolynomials.begin(); itpoly != _vpolynomials.end(); ++itpoly ) {
        vNewPolynomials.emplace_back( itpoly->Differentiate(ideriv) );
    }
    return PiecewisePolynomial(vNewPolynomials); // initial values of all polynomials will be updated.
}

PiecewisePolynomial PiecewisePolynomial::Integrate(const dReal c) const
{
    std::vector<Polynomial> vNewPolynomials;
    vNewPolynomials.reserve(_vpolynomials.size());
    for( std::vector<Polynomial>::const_iterator itpoly = _vpolynomials.begin(); itpoly != _vpolynomials.end(); ++itpoly ) {
        vNewPolynomials.emplace_back( itpoly->Integrate(0) );
    }
    vNewPolynomials.front().UpdateInitialValue(c);
    return PiecewisePolynomial(vNewPolynomials); // initial values of all polynomials will be updated.
}

void PiecewisePolynomial::Cut(dReal t, PiecewisePolynomial &remPWPolynomial)
{
    // TODO
    throw OPENRAVE_EXCEPTION_FORMAT0("Cut not implemented", ORE_NotImplemented);
}

void PiecewisePolynomial::TrimFront(dReal t)
{
    // TODO
    throw OPENRAVE_EXCEPTION_FORMAT0("TrimFront not implemented", ORE_NotImplemented);
}

void PiecewisePolynomial::TrimBack(dReal t)
{
    // TODO
    throw OPENRAVE_EXCEPTION_FORMAT0("TrimBack not implemented", ORE_NotImplemented);
}

Polynomial PiecewisePolynomial::ExtractPolynomial(const dReal t0, const dReal t1) const
{
    OPENRAVE_ASSERT_OP(t1, >, t0);
    size_t index0, index1;
    dReal remainder0, remainder1;
    FindPolynomialIndex(t0, index0, remainder0);
    while( _vpolynomials[index0].duration == 0 ) {
        ++index0;
    }
    FindPolynomialIndex(t1, index1, remainder1);
    if( index0 != index1 ) {
        try {
            OPENRAVE_ASSERT_OP(index0 + 1, ==, index1);
            BOOST_ASSERT( FuzzyZero(remainder1, g_fPolynomialEpsilon) );
        }
        catch( const std::exception& ex ) {
            std::stringstream ssdebug;
            ssdebug << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
            this->Serialize(ssdebug);
            RAVELOG_WARN_FORMAT("exception happened when extracting polynomial: %s\nt0=%.15e; t1=%.15e; index0=%d; remainder0=%.15e; index1=%d; remainder1=%.15e; pwpdata=\"\"\"%s\"\"\"", ex.what()%t0%t1%index0%remainder0%index1%remainder1%ssdebug.str());
            throw;
        }
    }
    Polynomial p(_vpolynomials[index0]);
    p.Reparameterize(remainder0);
    p.UpdateDuration(t1 - t0);
    return p;
}

void PiecewisePolynomial::CleanUp()
{
    size_t iWriteIndex = 0;
    for( size_t iReadIndex = 0; iReadIndex < _vpolynomials.size(); ++iReadIndex ) {
        if( _vpolynomials[iReadIndex].duration > g_fEpsilonForTimeInstant ) {
            if( iWriteIndex < iReadIndex ) {
                _vpolynomials[iWriteIndex] = _vpolynomials[iReadIndex];
            }
            ++iWriteIndex;
        }
    }
    if( iWriteIndex == 0 ) {
        iWriteIndex = 1;
    }
    _vpolynomials.resize(iWriteIndex);
    dReal duration = 0;
    for( std::vector<Polynomial>::const_iterator itpoly = _vpolynomials.begin(); itpoly != _vpolynomials.end(); ++itpoly ) {
        duration += itpoly->duration;
    }
    _duration = duration;
}

//
// Chunk
//
Chunk::Chunk(const dReal duration_, const std::vector<Polynomial>& vpolynomials_)
{
    Initialize(duration_, vpolynomials_);
}

void Chunk::UpdateInitialValues(std::vector<dReal>&vinitialvalues)
{
    OPENRAVE_ASSERT_OP(vinitialvalues.size(), ==, dof);
    for( size_t idof = 0; idof < dof; ++idof ) {
        vpolynomials[idof].UpdateInitialValue(vinitialvalues[idof]);
    }
}

void Chunk::UpdateDuration(dReal T)
{
    OPENRAVE_ASSERT_OP(T, >=, 0);
    duration = T;
    for( size_t idof = 0; idof < dof; ++idof ) {
        vpolynomials[idof].UpdateDuration(T);
    }
}

void Chunk::Initialize(const dReal duration_, const std::vector<Polynomial>& vpolynomials_)
{
    for( std::vector<Polynomial>::const_iterator itpoly = vpolynomials_.begin(); itpoly != vpolynomials_.end(); ++itpoly ) {
        if( !FuzzyEquals(itpoly->duration, duration_, g_fPolynomialEpsilon) ) {
            throw OPENRAVE_EXCEPTION_FORMAT("Polynomial index=%d has duration=%f (expected %f)", (itpoly - vpolynomials_.begin())%(itpoly->duration)%duration_, ORE_InvalidArguments);
        }
    }
    this->duration = duration_;
    this->vpolynomials = vpolynomials_;
    Initialize();
}

void Chunk::Initialize()
{
    this->dof = this->vpolynomials.size();
    this->degree = 0;
    for( size_t i = 0; i < vpolynomials.size(); ++i ) {
        if( vpolynomials[i].degree > this->degree ) {
            this->degree = vpolynomials[i].degree;
        }

        vpolynomials[i].UpdateDuration(duration);
    }
    for( size_t i = 1; i < this->vpolynomials.size(); ++i ) {
        if( this->vpolynomials[i].degree < this->degree ) {
            this->vpolynomials[i].PadCoefficients(this->degree);
        }
    }
    this->constraintChecked = false; // always set constraintChecked to false. setting it to true should be done explicitly outside.
}

void Chunk::Cut(dReal t, Chunk& remChunk)
{
    // TODO: handle edge cases for t.
    OPENRAVE_ASSERT_OP(t, >=, 0);
    OPENRAVE_ASSERT_OP(t, <=, duration);
    // TODO: Need to find a better way to not have to create these vectors every time this function is called.
    std::vector<Polynomial> vpoly = this->vpolynomials; // copy
    const dReal originalDuration = this->duration;
    for( size_t idof = 0; idof < this->dof; ++idof ) {
        vpoly[idof].Reparameterize(t); // so that vpoly[idof].Eval(0) = this->vpolynomials[idof].Eval(t)
        vpoly[idof].UpdateDuration(originalDuration - t);
    }
    remChunk.Initialize(originalDuration - t, vpoly);

    this->UpdateDuration(t);
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

void Chunk::SetConstant(const std::vector<dReal>& x0Vect, const dReal duration_, const size_t degree_)
{
    this->dof = x0Vect.size();
    this->degree = degree_;
    this->duration = duration_;
    vpolynomials.resize(this->dof);
    std::vector<dReal> vcoeffs(this->degree + 1, 0);
    for( size_t idof = 0; idof < this->dof; ++idof ) {
        // All other coefficients are zero.
        vcoeffs[0] = x0Vect[idof];
        vpolynomials[idof].Initialize(duration_, vcoeffs);
    }
}

//
// Piecewise Polynomial Trajectory
//
PiecewisePolynomialTrajectory::PiecewisePolynomialTrajectory(const std::vector<Chunk>& vchunks_)
{
    Initialize(vchunks_);
}

void PiecewisePolynomialTrajectory::Initialize(const std::vector<Chunk>& vchunks_)
{
    this->vchunks = vchunks_;
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

void PiecewisePolynomialTrajectory::ReplaceSegment(dReal t0, dReal t1, const std::vector<Chunk>& vchunks_)
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
        this->vchunks.insert(this->vchunks.begin() + index0 + 1, vchunks_.begin(), vchunks_.end()); // insert vchunks after index0
    }
    else {
        Chunk tempChunk;
        // Manage chunk index1
        this->vchunks[index1].Cut(rem1, tempChunk);
        this->vchunks[index1] = tempChunk; // replace chunk index1 by the portion from rem1 to duration

        // Manager chunk index0
        this->vchunks[index0].Cut(rem0, tempChunk); // TODO: maybe define TrimBack function is better

        // Insert the input chunks before index1
        this->vchunks.insert(this->vchunks.begin() + index1, vchunks_.begin(), vchunks_.end());

        // Remove the previous segment from index0 to index1
        this->vchunks.erase(this->vchunks.begin() + index0 + 1, this->vchunks.begin() + index1);
    }
    this->Initialize();
}

} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE
