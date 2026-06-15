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
#include "polynomialcommon.h"

namespace OpenRAVE {

namespace PiecewisePolynomialsInternal {

class Coordinate {
public:
    Coordinate()
    {
    }
    Coordinate(dReal p, dReal v) : point(p), value(v)
    {
    }
    ~Coordinate()
    {
    }
    bool operator<(const Coordinate& rhs) const
    {
        return point < rhs.point;
    }
    dReal point;
    dReal value;
}; // end class Coordinate

class Polynomial {
public:
    /*
       \params vcoeffs vector of polynomial coefficients, starting from the weakest term. That is,
               this polynomial p is described by p(t) = c[0] + c[1]*t + c[2]*t**2 + ... + c[n]*t**n,
               where n is the degree of this polynomial
     */
    Polynomial()
    {
    };
    Polynomial(const dReal T, const std::vector<dReal>& c);
    virtual ~Polynomial()
    {
    }

    //
    // Functions
    //
    /// \brief Initialize this polynomial with the given coefficients.
    void Initialize(const dReal T, const std::vector<dReal>& c);

    /// \brief Initialize this polynomial with the existing coefficients
    void Initialize();

    /// \brief Append zeros to the coefficient vectors.
    void PadCoefficients(const size_t newdegree);

    /// \brief Update the weakest term coefficient.
    void UpdateInitialValue(const dReal c0);

    /// \brief Update the duration.
    void UpdateDuration(const dReal T);

    /// \brief Reparameterize this polynomial p(t) into p(u) where u = t - t0. Suppose we originally
    ///        have p.Eval(t0) = c. After reparameterization p.Reparameterize(t0), we will have
    ///        p.Eval(0) = c.
    void Reparameterize(const dReal t0);

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

    /// \brief Return the polynomial d/dt p(t)
    Polynomial Differentiate(const size_t ideriv) const;

    /// \brief Evaluate the first integral of this polynomial at time t with integration constant c.
    dReal Evali1(dReal t, const dReal c=0) const;

    /// \brief Return the polynomial int p(t) dt
    Polynomial Integrate(const dReal c=0) const;

    /// \brief Return the vector of coordinates of extrema of this polynomial.
    const std::vector<Coordinate>& GetExtrema() const
    {
        if( !_bExtremaComputed ) {
            _FindAllLocalExtrema();
        }
        return vcextrema;
    }

    /// \brief Serialize this polynomial into stringstream.
    void Serialize(std::ostream& O) const;

    /// \brief Deserialize polynomial data from the input stream and initialize this polynomial with the data.
    void Deserialize(std::istream& I);

    /// \brief Find all local extrema of the ideriv-th derivative of this polynomial. Keep the
    /// results in vcoord in ascending order.
    void FindAllLocalExtrema(size_t ideriv, std::vector<Coordinate>& vcoord) const;

    virtual Polynomial& operator=(const Polynomial& r);

    //
    // Members
    //
    size_t degree; ///< the degree of this polynomial
    std::vector<dReal> vcoeffs; ///< vector of coefficients of this polynomial (weakest term first)
    // The following coefficient vectors will be computed only when needed
    mutable std::vector<dReal> vcoeffsd; ///< vector of coefficients of the first derivative of this polynomial
    mutable std::vector<dReal> vcoeffsdd; ///< vector of coefficients of the second derivative of this polynomial
    mutable std::vector<dReal> vcoeffsddd; ///< vector of coefficients of the third derivative of this polynomial

    dReal displacement; ///< the displacement done by this polynomial
    dReal duration; ///< the duration T of this polynomial. The polynomial p(t) is valid for t \in [0, T].

private:
    /// \brief Make sure _vcoeffd is initialized. Must be called before _vcoeffsd is used.
    void _EnsureInitCoeffs1() const;
    mutable bool _bInitCoeffs1=false; ///< if true, then _vcoeffsd is ready for use

    /// \brief Make sure _vcoeffdd is initialized. Must be called before _vcoeffsdd is used.
    void _EnsureInitCoeffs2() const;
    mutable bool _bInitCoeffs2=false; ///< if true, then _vcoeffsdd is ready for use

    /// \brief Make sure _vcoeffddd is initialized. Must be called before _vcoeffsddd is used.
    void _EnsureInitCoeffs3() const;
    mutable bool _bInitCoeffs3=false; ///< if true, then _vcoeffsddd is ready for use

    /// \brief Find all local extrema of this polynomial. Keep the results in vcextrema in ascending
    /// order.
    void _FindAllLocalExtrema() const;

    mutable std::vector<Coordinate> vcextrema; ///< vector of pairs (t, p(t)) where each t is such that p(t) is a local extremum.
    mutable bool _bExtremaComputed=false; ///< indicates whether the list of local extrema has already been computed. The only function that modifies this flag is _FindAllLocalExtrema.

    mutable std::vector<dReal> _vcurcoeffs;
}; // end class Polynomial

class PiecewisePolynomial {
public:
    /*
       PiecewisePolynomial is a horizontal stack of polynomials.

       \params polynomialsIn vector of polynomials
     */
    PiecewisePolynomial() {
    }
    PiecewisePolynomial(std::vector<Polynomial>& polynomialsIn);
    ~PiecewisePolynomial() {
    }

    void Initialize(std::vector<Polynomial>& polynomialsIn);
    void Initialize(Polynomial& polynomialIn);

    void Append(Polynomial& newPolynomial);

    /// \brief Update the weakest term coefficients of all the polynomials
    void UpdateInitialValue(const dReal c0);

    /// \brief Find the index of the polynomial q that t falls into and also compute the remainder
    ///        so that p(t) = q(remainder)
    void FindPolynomialIndex(dReal t, size_t& index, dReal& remainder) const;

    /// \brief Evaluate this piecewise polynomial at time t.
    dReal Eval(dReal t) const;

    /// \brief Evaluate the first derivative of this piecewise polynomial at time t.
    dReal Evald1(dReal t) const;

    /// \brief Evaluate the second derivative of this piecewise polynomial at time t.
    dReal Evald2(dReal t) const;

    /// \brief Evaluate the third derivative of this piecewise polynomial at time t.
    dReal Evald3(dReal t) const;

    /// \brief Evaluate the n-th derivative of this piecewise polynomial at time t.
    dReal Evaldn(dReal t, size_t n) const;

    /// \brief Evaluate the first integral of this piecewise polynomial at time t with integration constant c.
    dReal Evali1(dReal t, const dReal c=0) const;

    /// \brief Serialize this piecewise-polynomial into the stringstream.
    void Serialize(std::ostream& O) const;

    /// \brief Deserialize piecewise-polynomial data from the input stream and initialize this piecewise-polynomial with the data.
    void Deserialize(std::istream& I);

    /// \brief Return the polynomial d/dt p(t)
    PiecewisePolynomial Differentiate(const size_t ideriv) const;

    /// \brief Return the polynomial int p(t) dt
    PiecewisePolynomial Integrate(const dReal c=0) const;

    /// \brief Cut the piecewise polynomial into two halves at time t. The left half is stored in the same
    /// piecewise polynomial. The right half is returned via remPWPolynomial
    void Cut(dReal t, PiecewisePolynomial &remPWPolynomial);

    /// \brief Cut the piecewise polynomial into two halves at time t and keep the right half.
    void TrimFront(dReal t);

    /// \brief Cut the piecewise polynomial into two halves at time t and keep the left half.
    void TrimBack(dReal t);

    /// \brief Remove from _vpolynomials polynomials that have zero duration. However, if all polynomials have zero
    ///        durations, will leave one polynomial in _vpolynomials so that _vpolynomials is not empty.
    void CleanUp();

    /// \brief Return a constant reference to _vpolynomials
    inline const std::vector<Polynomial>& GetPolynomials() const
    {
        return _vpolynomials;
    }

    /// \brief Return a const reference to _vpolynomials[index]
    inline const Polynomial& GetPolynomial(size_t index) const
    {
        return _vpolynomials.at(index);
    }

    /// \brief Get the total duration of the curve
    inline const dReal GetDuration() const
    {
        return _duration;
    }

    /// \brief Return a new polynomial representing a segment starting from t0 and ending at t1. t0
    ///        and t1 must be on the same polynomial, i.e.  FindPolynomialIndex(t0) and
    ///        FindPolynomialIndex(t1) returns the same polynomial index.
    Polynomial ExtractPolynomial(const dReal t0, const dReal t1) const;

private:
    dReal _duration; ///< the total duration of this piecewise polynomial
    std::vector<Polynomial> _vpolynomials;
}; // end class PiecewisePolynomial

class Chunk {
public:
    /*
       Chunk is a vertical stack of polynomials. Each polynomial is parameterized by t \in [0, duration].

       \params duration the duration of this chunk
       \params vpolynomials vector of polynomials
     */
    Chunk() : duration(0), constraintChecked(false)
    {
    }
    Chunk(const dReal duration, const std::vector<Polynomial>& vpolynomials);
    ~Chunk()
    {
    }

    //
    // Functions
    //
    /// \brief Update the initial values of all polynomials according to vinitialvalues.
    void UpdateInitialValues(std::vector<dReal>& vinitialvalues);

    /// \brief Update the duration of all polynomials.
    void UpdateDuration(dReal T);

    /// \brief Initialize this chunk with the given duration and polynomials vector
    void Initialize(const dReal duration, const std::vector<Polynomial>& vpolynomials);

    /// \brief Initialize this chunk with the existing duration and polynomials vector
    void Initialize();

    /// \brief Cut this chunk into two halves. The left half (from t = 0 to t = t) is stored in
    /// this. The right half is returned via remChunk.
    void Cut(dReal t, Chunk& remChunk);

    /// \brief Evaluate all polynomials at time t.
    void Eval(dReal t, std::vector<dReal>& res) const;

    /// \brief Evaluate the first derivatives of all polynomials at time t.
    void Evald1(dReal t, std::vector<dReal>& res) const;

    /// \brief Evaluate the second derivatives of all polynomials at time t.
    void Evald2(dReal t, std::vector<dReal>& res) const;

    /// \brief Evaluate the third derivatives of all polynomials at time t.
    void Evald3(dReal t, std::vector<dReal>& res) const;

    /// \brief Evaluate the n-th derivatives of all polynomials at time t.
    void Evaldn(dReal t, size_t n, std::vector<dReal>& res) const;

    /// \brief Serialize this chunk into stringstream.
    void Serialize(std::ostream& O) const;

    /// \brief Deserialize chunk data from the input stream and initialize this chunk with the data.
    void Deserialize(std::istream& I);

    /// \brief Initialize this chunk with constant polynomials.
    void SetConstant(const std::vector<dReal>& x0Vect, const dReal duration, const size_t degree);

    //
    // Members
    //
    size_t degree;
    size_t dof;
    dReal duration;
    std::vector<Polynomial> vpolynomials;

    mutable bool constraintChecked = false; ///< TODO: write a description for this parameter (similar to that of RampND)
    mutable int _iteration = -1; /// for debugging only. keeps track of the shortcut iteration from which this chunk is introduced into the final trajectory.

}; // end class Chunk

class PiecewisePolynomialTrajectory {
public:
    /*
       PiecewisePolynomialTrajectory is a horizontal stack of chunks.

       \params vchunks vector of chunks
     */
    PiecewisePolynomialTrajectory()
    {
    }
    PiecewisePolynomialTrajectory(const std::vector<Chunk>& vchunks);
    ~PiecewisePolynomialTrajectory()
    {
    }

    //
    // Functions
    //
    /// \brief Evaluate this trajectory at time t.
    void Eval(dReal t, std::vector<dReal>& res) const;

    /// \brief Evaluate the first derivative of this trajectory at time t.
    void Evald1(dReal t, std::vector<dReal>& res) const;

    /// \brief Evaluate the second derivative of this trajectory at time t.
    void Evald2(dReal t, std::vector<dReal>& res) const;

    /// \brief Evaluate the third derivative of this trajectory at time t.
    void Evald3(dReal t, std::vector<dReal>& res) const;

    /// \brief Evaluate the n-th derivative of this trajectory at time t.
    void Evaldn(dReal t, size_t n, std::vector<dReal>& res) const;

    /// \brief Find the index of the chunk in which the given time t falls into. Also compute the remainder of that chunk.
    void FindChunkIndex(dReal t, size_t& index, dReal& remainder) const;

    /// \brief Initialize this trajectory with the given chunks
    void Initialize(const std::vector<Chunk>& vchunks);

    /// \brief Initializee this trajectory with the existing chunks
    void Initialize();

    /// \brief Process the vector of chunks so that the initial values of each chunk is equal to the
    ///        final values of the preceding one.
    void _UpdateChunksVector();

    /// \brief Serialize this piecewise-polynomial trajectory into stringstream.
    void Serialize(std::ostream& O) const;

    /// \brief Deserialize trajectory data from the input stream and initialize this trajectory with the data.
    void Deserialize(std::istream& I);

    /// \brief Replace the content of this trajectory starting from t = t0 to t = t1 with the new
    ///        content given by vchunks.
    ///        This function does not check continuity at the junctions.
    void ReplaceSegment(dReal t0, dReal t1, const std::vector<Chunk>& vchunks);

    /// \brief Reset the trajectory data.
    inline void Reset()
    {
        degree = 0;
        dof = 0;
        duration = 0;
        vswitchtimes.resize(0);
        vchunks.resize(0);
    }

    //
    // Members
    //
    size_t degree;
    size_t dof;
    dReal duration;
    std::vector<dReal> vswitchtimes; ///< vector of time instants where at least one dof changes its control value. The control value is p^(degree)(t)/degree!. For example, in case degree = 2, the control value is p''(t)/2, which is the acceleration.
    std::vector<Chunk> vchunks;

}; // end class PiecewisePolynomialTrajectory

typedef boost::shared_ptr<Coordinate> CoordinatePtr;
typedef boost::shared_ptr<Polynomial> PolynomialPtr;
typedef boost::shared_ptr<PiecewisePolynomial> PiecewisePolynomialPtr;
typedef boost::shared_ptr<Chunk> ChunkPtr;
typedef boost::shared_ptr<PiecewisePolynomialTrajectory> PiecewisePolynomialTrajectoryPtr;

} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE
#endif
