// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
/** \file   mathextra.h
    \brief  Extra math routines that are useful to have but don't really belong anywhere.
 */
#ifndef OPENRAVE_MATHEXTRA_H
#define OPENRAVE_MATHEXTRA_H

#ifndef OPENRAVE_API
#define OPENRAVE_API
#endif

#ifdef BOOST_ASSERT
#define MATH_ASSERT BOOST_ASSERT
#else
#include <cassert>
#define MATH_ASSERT assert
#endif

#include <cmath>
#include <climits>

namespace OpenRAVE {

/// Extra math routines that are useful to have but don't really belong anywhere.
namespace mathextra {

#define distinctRoots 0                       // roots r0 < r1 < r2
#define singleRoot 1                       // root r0
#define floatRoot01 2                       // roots r0 = r1 < r2
#define floatRoot12 4                       // roots r0 < r1 = r2
#define tripleRoot 6                       // roots r0 = r1 = r2

// multiplies 4x4 matrices
inline float* mult4(float* pfres, const float* pf1, const float* pf2);

// pf1^T * pf2
inline float* multtrans3(float* pfres, const float* pf1, const float* pf2);
inline float* multtrans4(float* pfres, const float* pf1, const float* pf2);
inline float* transnorm3(float* pfout, const float* pfmat, const float* pf);

inline float* transpose3(const float* pf, float* pfres);
inline float* transpose4(const float* pf, float* pfres);

inline float dot2(const float* pf1, const float* pf2);
inline float dot3(const float* pf1, const float* pf2);
inline float dot4(const float* pf1, const float* pf2);

inline float lengthsqr2(const float* pf);
inline float lengthsqr3(const float* pf);
inline float lengthsqr4(const float* pf);

inline float* normalize2(float* pfout, const float* pf);
inline float* normalize3(float* pfout, const float* pf);
inline float* normalize4(float* pfout, const float* pf);

inline float* cross3(float* pfout, const float* pf1, const float* pf2);

// multiplies 3x3 matrices
inline float* mult3_s4(float* pfres, const float* pf1, const float* pf2);
inline float* mult3_s3(float* pfres, const float* pf1, const float* pf2);

inline float* inv3(const float* pf, float* pfres, float* pfdet, int stride);
inline float* inv4(const float* pf, float* pfres);


inline double* mult4(double* pfres, const double* pf1, const double* pf2);

// pf1^T * pf2
inline double* multtrans3(double* pfres, const double* pf1, const double* pf2);
inline double* multtrans4(double* pfres, const double* pf1, const double* pf2);
inline double* transnorm3(double* pfout, const double* pfmat, const double* pf);

inline double* transpose3(const double* pf, double* pfres);
inline double* transpose4(const double* pf, double* pfres);

inline double dot2(const double* pf1, const double* pf2);
inline double dot3(const double* pf1, const double* pf2);
inline double dot4(const double* pf1, const double* pf2);

inline double lengthsqr2(const double* pf);
inline double lengthsqr3(const double* pf);
inline double lengthsqr4(const double* pf);

inline double* normalize2(double* pfout, const double* pf);
inline double* normalize3(double* pfout, const double* pf);
inline double* normalize4(double* pfout, const double* pf);

inline double* cross3(double* pfout, const double* pf1, const double* pf2);

// multiplies 3x3 matrices
inline double* mult3_s4(double* pfres, const double* pf1, const double* pf2);
inline double* mult3_s3(double* pfres, const double* pf1, const double* pf2);

inline double* inv3(const double* pf, double* pfres, double* pfdet, int stride);
inline double* inv4(const double* pf, double* pfres);


////
// More complex ops that deal with arbitrary matrices //
////

/// extract eigen values and vectors from a 2x2 matrix and returns true if all values are real
/// returned eigen vectors are normalized
template <typename T>
inline bool eig2(const T* pfmat, T* peigs, T& fv1x, T& fv1y, T& fv2x, T& fv2y);

// Simple routines for linear algebra algorithms //

OPENRAVE_API int CubicRoots (double c0, double c1, double c2, double *r0, double *r1, double *r2);
template <typename T, typename S> void Tridiagonal3 (S* mat, T* diag, T* subd);
OPENRAVE_API bool QLAlgorithm3 (float* m_aafEntry, float* afDiag, float* afSubDiag);
OPENRAVE_API bool QLAlgorithm3 (double* m_aafEntry, double* afDiag, double* afSubDiag);
OPENRAVE_API void EigenSymmetric3(const double* fCovariance, double* eval, double* fAxes);

/// Computes the eigenvectors of the covariance matrix and forms a basis
/// \param[in] fCovariance a symmetric 3x3 matrix.
/// \param[out] vbasis the basis vectors extracted (form a right hand coordinate system).
template <typename T>
inline void GetCovarBasisVectors(const T fCovariance[3][3], T vbasis[3][3])
{
    T EigenVals[3];
    EigenSymmetric3((const T*)fCovariance, EigenVals, (T*)vbasis);
    // make sure that the new axes follow the right-hand coord system
    normalize3(vbasis[0],vbasis[0]);
    T f = dot3(vbasis[0],vbasis[1]);
    vbasis[1][0] -= vbasis[0][0]*f; vbasis[1][1] -= vbasis[0][1]*f; vbasis[1][2] -= vbasis[0][2]*f;
    normalize3(vbasis[1],vbasis[1]);
    cross3(vbasis[2],vbasis[0],vbasis[1]);
}

/// SVD of a 3x3 matrix A such that A = U*diag(D)*V'
/// The row stride for all matrices is 3*sizeof(T) bytes
/// \param[in] A 3x3 matrix
/// \param[out] U 3x3 matrix
/// \param[out] D 3x1 matrix
/// \param[out] V 3x3 matrix
template <typename T> inline void svd3(const T* A, T* U, T* D, T* V);

template <typename T> inline void mult(T* pf, T fa, int r)
{
    MATH_ASSERT( pf != NULL );
    while(r > 0) {
        --r;
        pf[r] *= fa;
    }
}

template <typename T> int Min(T* pts, int stride, int numPts); // returns the index, stride in units of T
template <typename T> int Max(T* pts, int stride, int numPts); // returns the index

// multiplies a matrix by a scalar
template <typename T> inline void mult(T* pf, T fa, int r);

// multiplies a r1xc1 by c1xc2 matrix into pfres, if badd is true adds the result to pfres
// does not handle cases where pfres is equal to pf1 or pf2, use multtox for those cases
template <typename T, typename R, typename S>
inline S* mult(T* pf1, R* pf2, int r1, int c1, int c2, S* pfres, bool badd = false);

// pf1 is transposed before mult
// rows of pf2 must equal rows of pf1
// pfres will be c1xc2 matrix
template <typename T, typename R, typename S>
inline S* multtrans(T* pf1, R* pf2, int r1, int c1, int c2, S* pfres, bool badd = false);

// pf2 is transposed before mult
// the columns of both matrices must be the same and equal to c1
// r2 is the number of rows in pf2
// pfres must be an r1xr2 matrix
template <typename T, typename R, typename S>
inline S* multtrans_to2(T* pf1, R* pf2, int r1, int c1, int r2, S* pfres, bool badd = false);

// multiplies rxc matrix pf1 and cxc matrix pf2 and stores the result in pf1,
// the function needs a temporary buffer the size of c doubles, if pftemp == NULL,
// the function will allocate the necessary memory, otherwise pftemp should be big
// enough to store all the entries
template <typename T> inline T* multto1(T* pf1, T* pf2, int r1, int c1, T* pftemp = NULL);

// same as multto1 except stores the result in pf2, pf1 has to be an r2xr2 matrix
// pftemp must be of size r2 if not NULL
template <typename T, typename S> inline T* multto2(T* pf1, S* pf2, int r2, int c2, S* pftemp = NULL);

// add pf1 + pf2 and store in pf1
template <typename T> inline void sub(T* pf1, T* pf2, int r);
template <typename T> inline T normsqr(const T* pf1, int r);
template <typename T> inline T lengthsqr(const T* pf1, const T* pf2, int length);
template <typename T> inline T dot(T* pf1, T* pf2, int length);

template <typename T> inline T sum(T* pf, int length);

/// takes the inverse of the 2x2 matrix pf and stores it into pfres, returns true if matrix is invertible
template <typename T> inline bool inv2(T* pf, T* pfres);

///////////////////////
// Function Definitions
///////////////////////
template <typename T>
bool eig2(const T* pfmat, T* peigs, T& fv1x, T& fv1y, T& fv2x, T& fv2y)
{
    // x^2 + bx + c
    T a, b, c, d;
    b = -(pfmat[0] + pfmat[3]);
    c = pfmat[0] * pfmat[3] - pfmat[1] * pfmat[2];
    d = b * b - 4.0f * c + 1e-16f;
    if( d < 0 ) {
        return false;
    }
    if( d < 1e-16f ) {
        a = -0.5f * b;
        peigs[0] = a;
        peigs[1] = a;
        fv1x = pfmat[1];
        fv1y = a - pfmat[0];
        c = 1 / sqrt(fv1x*fv1x + fv1y*fv1y);
        fv1x *= c;
        fv1y *= c;
        fv2x = -fv1y;
        fv2y = fv1x;
        return true;
    }
    // two roots
    d = sqrt(d);
    a = -0.5f * (b + d);
    peigs[0] = a;
    fv1x = pfmat[1];
    fv1y = a-pfmat[0];
    c = 1 / sqrt(fv1x*fv1x + fv1y*fv1y);
    fv1x *= c;
    fv1y *= c;
    a += d;
    peigs[1] = a;
    fv2x = pfmat[1];
    fv2y = a-pfmat[0];
    c = 1 / sqrt(fv2x*fv2x + fv2y*fv2y);
    fv2x *= c;
    fv2y *= c;
    return true;
}

// returns the number of real roots, fills r1 and r2 with the answers
template <typename T>
inline int solvequad(T a, T b, T c, T& r1, T& r2)
{
    if(a == 0) {
        if(b == 0) {
            if(c == 0) {
                return 0;
            }
            // invalid equation
            return 0;
        }
        r1=r2=-c/b;
        return 1;
    }
    
    T d = b * b - (T)4 * c * a;
    if( d < 0 ) {
        if( d < -(T)1e-16) {
            return 0;
        }
        // d is close to 0, so most likely floating precision error
        d = 0;
    }
    if( d < (T)1e-16 ) {
        r1 = r2 = (T)-0.5 * b / a;
        return 1;
    }
    // two roots. need to explicitly divide by a to preserve precision
    d = sqrt(d);
    if(fabs(-b - d) < fabs(a)) {
        r1 = 0.5 * (-b + d)/a;
    }
    else {
        r1 = 2.0 * c / (-b-d);
    }
    if(fabs(-b + d) < fabs(a)) {
        r2 = 0.5 * (-b-d) / a;
    }
    else {
        r2 = 2.0 * c / (-b+d);
    }
    return 2;
}

#define MULT3(stride) { \
        pfres2[0*stride+0] = pf1[0*stride+0]*pf2[0*stride+0]+pf1[0*stride+1]*pf2[1*stride+0]+pf1[0*stride+2]*pf2[2*stride+0]; \
        pfres2[0*stride+1] = pf1[0*stride+0]*pf2[0*stride+1]+pf1[0*stride+1]*pf2[1*stride+1]+pf1[0*stride+2]*pf2[2*stride+1]; \
        pfres2[0*stride+2] = pf1[0*stride+0]*pf2[0*stride+2]+pf1[0*stride+1]*pf2[1*stride+2]+pf1[0*stride+2]*pf2[2*stride+2]; \
        pfres2[1*stride+0] = pf1[1*stride+0]*pf2[0*stride+0]+pf1[1*stride+1]*pf2[1*stride+0]+pf1[1*stride+2]*pf2[2*stride+0]; \
        pfres2[1*stride+1] = pf1[1*stride+0]*pf2[0*stride+1]+pf1[1*stride+1]*pf2[1*stride+1]+pf1[1*stride+2]*pf2[2*stride+1]; \
        pfres2[1*stride+2] = pf1[1*stride+0]*pf2[0*stride+2]+pf1[1*stride+1]*pf2[1*stride+2]+pf1[1*stride+2]*pf2[2*stride+2]; \
        pfres2[2*stride+0] = pf1[2*stride+0]*pf2[0*stride+0]+pf1[2*stride+1]*pf2[1*stride+0]+pf1[2*stride+2]*pf2[2*stride+0]; \
        pfres2[2*stride+1] = pf1[2*stride+0]*pf2[0*stride+1]+pf1[2*stride+1]*pf2[1*stride+1]+pf1[2*stride+2]*pf2[2*stride+1]; \
        pfres2[2*stride+2] = pf1[2*stride+0]*pf2[0*stride+2]+pf1[2*stride+1]*pf2[1*stride+2]+pf1[2*stride+2]*pf2[2*stride+2]; \
}

/// mult3 with a 3x3 matrix whose row stride is 16 bytes
template <typename T>
inline T* _mult3_s4(T* pfres, const T* pf1, const T* pf2)
{
    MATH_ASSERT( pf1 != NULL && pf2 != NULL && pfres != NULL );

    T* pfres2;
    if((pfres == pf1)||(pfres == pf2)) pfres2 = (T*)alloca(12 * sizeof(T));
    else pfres2 = pfres;

    MULT3(4);
    if( pfres2 != pfres ) memcpy(pfres, pfres2, 12*sizeof(T));
    return pfres;
}

/// mult3 with a 3x3 matrix whose row stride is 12 bytes
template <typename T>
inline T* _mult3_s3(T* pfres, const T* pf1, const T* pf2)
{
    MATH_ASSERT( pf1 != NULL && pf2 != NULL && pfres != NULL );

    T* pfres2;
    if((pfres == pf1)||(pfres == pf2)) pfres2 = (T*)alloca(9 * sizeof(T));
    else pfres2 = pfres;

    MULT3(3);

    if( pfres2 != pfres ) memcpy(pfres, pfres2, 9*sizeof(T));

    return pfres;
}

// mult4
template <typename T>
inline T* _mult4(T* pfres, const T* p1, const T* p2)
{
    MATH_ASSERT( pfres != NULL && p1 != NULL && p2 != NULL );

    T* pfres2;
    if((pfres == p1)||(pfres == p2)) pfres2 = (T*)alloca(16 * sizeof(T));
    else pfres2 = pfres;

    pfres2[0*4+0] = p1[0*4+0]*p2[0*4+0] + p1[0*4+1]*p2[1*4+0] + p1[0*4+2]*p2[2*4+0] + p1[0*4+3]*p2[3*4+0];
    pfres2[0*4+1] = p1[0*4+0]*p2[0*4+1] + p1[0*4+1]*p2[1*4+1] + p1[0*4+2]*p2[2*4+1] + p1[0*4+3]*p2[3*4+1];
    pfres2[0*4+2] = p1[0*4+0]*p2[0*4+2] + p1[0*4+1]*p2[1*4+2] + p1[0*4+2]*p2[2*4+2] + p1[0*4+3]*p2[3*4+2];
    pfres2[0*4+3] = p1[0*4+0]*p2[0*4+3] + p1[0*4+1]*p2[1*4+3] + p1[0*4+2]*p2[2*4+3] + p1[0*4+3]*p2[3*4+3];

    pfres2[1*4+0] = p1[1*4+0]*p2[0*4+0] + p1[1*4+1]*p2[1*4+0] + p1[1*4+2]*p2[2*4+0] + p1[1*4+3]*p2[3*4+0];
    pfres2[1*4+1] = p1[1*4+0]*p2[0*4+1] + p1[1*4+1]*p2[1*4+1] + p1[1*4+2]*p2[2*4+1] + p1[1*4+3]*p2[3*4+1];
    pfres2[1*4+2] = p1[1*4+0]*p2[0*4+2] + p1[1*4+1]*p2[1*4+2] + p1[1*4+2]*p2[2*4+2] + p1[1*4+3]*p2[3*4+2];
    pfres2[1*4+3] = p1[1*4+0]*p2[0*4+3] + p1[1*4+1]*p2[1*4+3] + p1[1*4+2]*p2[2*4+3] + p1[1*4+3]*p2[3*4+3];

    pfres2[2*4+0] = p1[2*4+0]*p2[0*4+0] + p1[2*4+1]*p2[1*4+0] + p1[2*4+2]*p2[2*4+0] + p1[2*4+3]*p2[3*4+0];
    pfres2[2*4+1] = p1[2*4+0]*p2[0*4+1] + p1[2*4+1]*p2[1*4+1] + p1[2*4+2]*p2[2*4+1] + p1[2*4+3]*p2[3*4+1];
    pfres2[2*4+2] = p1[2*4+0]*p2[0*4+2] + p1[2*4+1]*p2[1*4+2] + p1[2*4+2]*p2[2*4+2] + p1[2*4+3]*p2[3*4+2];
    pfres2[2*4+3] = p1[2*4+0]*p2[0*4+3] + p1[2*4+1]*p2[1*4+3] + p1[2*4+2]*p2[2*4+3] + p1[2*4+3]*p2[3*4+3];

    pfres2[3*4+0] = p1[3*4+0]*p2[0*4+0] + p1[3*4+1]*p2[1*4+0] + p1[3*4+2]*p2[2*4+0] + p1[3*4+3]*p2[3*4+0];
    pfres2[3*4+1] = p1[3*4+0]*p2[0*4+1] + p1[3*4+1]*p2[1*4+1] + p1[3*4+2]*p2[2*4+1] + p1[3*4+3]*p2[3*4+1];
    pfres2[3*4+2] = p1[3*4+0]*p2[0*4+2] + p1[3*4+1]*p2[1*4+2] + p1[3*4+2]*p2[2*4+2] + p1[3*4+3]*p2[3*4+2];
    pfres2[3*4+3] = p1[3*4+0]*p2[0*4+3] + p1[3*4+1]*p2[1*4+3] + p1[3*4+2]*p2[2*4+3] + p1[3*4+3]*p2[3*4+3];

    if( pfres != pfres2 ) memcpy(pfres, pfres2, sizeof(T)*16);
    return pfres;
}

template <typename T>
inline T* _multtrans3(T* pfres, const T* pf1, const T* pf2)
{
    T* pfres2;
    if( pfres == pf1 ) pfres2 = (T*)alloca(9 * sizeof(T));
    else pfres2 = pfres;

    pfres2[0] = pf1[0]*pf2[0]+pf1[3]*pf2[3]+pf1[6]*pf2[6];
    pfres2[1] = pf1[0]*pf2[1]+pf1[3]*pf2[4]+pf1[6]*pf2[7];
    pfres2[2] = pf1[0]*pf2[2]+pf1[3]*pf2[5]+pf1[6]*pf2[8];

    pfres2[3] = pf1[1]*pf2[0]+pf1[4]*pf2[3]+pf1[7]*pf2[6];
    pfres2[4] = pf1[1]*pf2[1]+pf1[4]*pf2[4]+pf1[7]*pf2[7];
    pfres2[5] = pf1[1]*pf2[2]+pf1[4]*pf2[5]+pf1[7]*pf2[8];

    pfres2[6] = pf1[2]*pf2[0]+pf1[5]*pf2[3]+pf1[8]*pf2[6];
    pfres2[7] = pf1[2]*pf2[1]+pf1[5]*pf2[4]+pf1[8]*pf2[7];
    pfres2[8] = pf1[2]*pf2[2]+pf1[5]*pf2[5]+pf1[8]*pf2[8];

    if( pfres2 != pfres ) memcpy(pfres, pfres2, 9*sizeof(T));

    return pfres;
}

template <typename T>
inline T* _multtrans4(T* pfres, const T* pf1, const T* pf2)
{
    T* pfres2;
    if( pfres == pf1 ) pfres2 = (T*)alloca(16 * sizeof(T));
    else pfres2 = pfres;

    for(int i = 0; i < 4; ++i) {
        for(int j = 0; j < 4; ++j) {
            pfres2[4*i+j] = pf1[i] * pf2[j] + pf1[i+4] * pf2[j+4] + pf1[i+8] * pf2[j+8] + pf1[i+12] * pf2[j+12];
        }
    }

    if( pfres2 != pfres ) memcpy(pfres, pfres2, 16*sizeof(T));

    return pfres;
}

/// \brief Compute the determinant of a 3x3 matrix whose row stride stride elements.
template <typename T> inline T matrixdet3(const T* pf, int stride)
{
    return pf[0*stride+2] * (pf[1*stride + 0] * pf[2*stride + 1] - pf[1*stride + 1] * pf[2*stride + 0]) +
           pf[1*stride+2] * (pf[0*stride + 1] * pf[2*stride + 0] - pf[0*stride + 0] * pf[2*stride + 1]) +
           pf[2*stride+2] * (pf[0*stride + 0] * pf[1*stride + 1] - pf[0*stride + 1] * pf[1*stride + 0]);
}

/** \brief 3x3 matrix inverse.

    \param[in] pf the input 3x3 matrix
    \param[out] pf the result of the operation, can be the same matrix as pf
    \param[out] pfdet if not NULL, fills it with the determinant of the source matrix
    \param[in] stride the stride in elements between elements.
 */
template <typename T>
inline T* _inv3(const T* pf, T* pfres, T* pfdet, int stride)
{
    T* pfres2;
    if( pfres == pf ) pfres2 = (T*)alloca(3 * stride * sizeof(T));
    else pfres2 = pfres;

    // inverse = C^t / det(pf) where C is the matrix of coefficients

    // calc C^t
    pfres2[0*stride + 0] = pf[1*stride + 1] * pf[2*stride + 2] - pf[1*stride + 2] * pf[2*stride + 1];
    pfres2[0*stride + 1] = pf[0*stride + 2] * pf[2*stride + 1] - pf[0*stride + 1] * pf[2*stride + 2];
    pfres2[0*stride + 2] = pf[0*stride + 1] * pf[1*stride + 2] - pf[0*stride + 2] * pf[1*stride + 1];
    pfres2[1*stride + 0] = pf[1*stride + 2] * pf[2*stride + 0] - pf[1*stride + 0] * pf[2*stride + 2];
    pfres2[1*stride + 1] = pf[0*stride + 0] * pf[2*stride + 2] - pf[0*stride + 2] * pf[2*stride + 0];
    pfres2[1*stride + 2] = pf[0*stride + 2] * pf[1*stride + 0] - pf[0*stride + 0] * pf[1*stride + 2];
    pfres2[2*stride + 0] = pf[1*stride + 0] * pf[2*stride + 1] - pf[1*stride + 1] * pf[2*stride + 0];
    pfres2[2*stride + 1] = pf[0*stride + 1] * pf[2*stride + 0] - pf[0*stride + 0] * pf[2*stride + 1];
    pfres2[2*stride + 2] = pf[0*stride + 0] * pf[1*stride + 1] - pf[0*stride + 1] * pf[1*stride + 0];

    T fdet = pf[0*stride + 2] * pfres2[2*stride + 0] + pf[1*stride + 2] * pfres2[2*stride + 1] +
             pf[2*stride + 2] * pfres2[2*stride + 2];

    if( pfdet != NULL )
        pfdet[0] = fdet;

    if( fabs(fdet) < 1e-12 ) {
        return NULL;
    }

    fdet = 1 / fdet;
    //if( pfdet != NULL ) *pfdet = fdet;

    if( pfres != pf ) {
        pfres[0*stride+0] *= fdet;              pfres[0*stride+1] *= fdet;              pfres[0*stride+2] *= fdet;
        pfres[1*stride+0] *= fdet;              pfres[1*stride+1] *= fdet;              pfres[1*stride+2] *= fdet;
        pfres[2*stride+0] *= fdet;              pfres[2*stride+1] *= fdet;              pfres[2*stride+2] *= fdet;
        return pfres;
    }

    pfres[0*stride+0] = pfres2[0*stride+0] * fdet;
    pfres[0*stride+1] = pfres2[0*stride+1] * fdet;
    pfres[0*stride+2] = pfres2[0*stride+2] * fdet;
    pfres[1*stride+0] = pfres2[1*stride+0] * fdet;
    pfres[1*stride+1] = pfres2[1*stride+1] * fdet;
    pfres[1*stride+2] = pfres2[1*stride+2] * fdet;
    pfres[2*stride+0] = pfres2[2*stride+0] * fdet;
    pfres[2*stride+1] = pfres2[2*stride+1] * fdet;
    pfres[2*stride+2] = pfres2[2*stride+2] * fdet;
    return pfres;
}

/// \brief 4x4 matrix inverse.
template <typename T>
inline T* _inv4(const T* pf, T* pfres)
{
    T* pfres2;
    if( pfres == pf ) pfres2 = (T*)alloca(16 * sizeof(T));
    else pfres2 = pfres;

    // inverse = C^t / det(pf) where C is the matrix of coefficients

    // calc C^t

    // determinants of all possibel 2x2 submatrices formed by last two rows
    T fd0, fd1, fd2;
    T f1, f2, f3;
    fd0 = pf[2*4 + 0] * pf[3*4 + 1] - pf[2*4 + 1] * pf[3*4 + 0];
    fd1 = pf[2*4 + 1] * pf[3*4 + 2] - pf[2*4 + 2] * pf[3*4 + 1];
    fd2 = pf[2*4 + 2] * pf[3*4 + 3] - pf[2*4 + 3] * pf[3*4 + 2];

    f1 = pf[2*4 + 1] * pf[3*4 + 3] - pf[2*4 + 3] * pf[3*4 + 1];
    f2 = pf[2*4 + 0] * pf[3*4 + 3] - pf[2*4 + 3] * pf[3*4 + 0];
    f3 = pf[2*4 + 0] * pf[3*4 + 2] - pf[2*4 + 2] * pf[3*4 + 0];

    pfres2[0*4 + 0] =   pf[1*4 + 1] * fd2 - pf[1*4 + 2] * f1 + pf[1*4 + 3] * fd1;
    pfres2[0*4 + 1] = -(pf[0*4 + 1] * fd2 - pf[0*4 + 2] * f1 + pf[0*4 + 3] * fd1);

    pfres2[1*4 + 0] = -(pf[1*4 + 0] * fd2 - pf[1*4 + 2] * f2 + pf[1*4 + 3] * f3);
    pfres2[1*4 + 1] =   pf[0*4 + 0] * fd2 - pf[0*4 + 2] * f2 + pf[0*4 + 3] * f3;

    pfres2[2*4 + 0] =   pf[1*4 + 0] * f1 -  pf[1*4 + 1] * f2 + pf[1*4 + 3] * fd0;
    pfres2[2*4 + 1] = -(pf[0*4 + 0] * f1 -  pf[0*4 + 1] * f2 + pf[0*4 + 3] * fd0);

    pfres2[3*4 + 0] = -(pf[1*4 + 0] * fd1 - pf[1*4 + 1] * f3 + pf[1*4 + 2] * fd0);
    pfres2[3*4 + 1] =   pf[0*4 + 0] * fd1 - pf[0*4 + 1] * f3 + pf[0*4 + 2] * fd0;

    // determinants of first 2 rows of 4x4 matrix
    fd0 = pf[0*4 + 0] * pf[1*4 + 1] - pf[0*4 + 1] * pf[1*4 + 0];
    fd1 = pf[0*4 + 1] * pf[1*4 + 2] - pf[0*4 + 2] * pf[1*4 + 1];
    fd2 = pf[0*4 + 2] * pf[1*4 + 3] - pf[0*4 + 3] * pf[1*4 + 2];

    f1 = pf[0*4 + 1] * pf[1*4 + 3] - pf[0*4 + 3] * pf[1*4 + 1];
    f2 = pf[0*4 + 0] * pf[1*4 + 3] - pf[0*4 + 3] * pf[1*4 + 0];
    f3 = pf[0*4 + 0] * pf[1*4 + 2] - pf[0*4 + 2] * pf[1*4 + 0];

    pfres2[0*4 + 2] =   pf[3*4 + 1] * fd2 - pf[3*4 + 2] * f1 + pf[3*4 + 3] * fd1;
    pfres2[0*4 + 3] = -(pf[2*4 + 1] * fd2 - pf[2*4 + 2] * f1 + pf[2*4 + 3] * fd1);

    pfres2[1*4 + 2] = -(pf[3*4 + 0] * fd2 - pf[3*4 + 2] * f2 + pf[3*4 + 3] * f3);
    pfres2[1*4 + 3] =   pf[2*4 + 0] * fd2 - pf[2*4 + 2] * f2 + pf[2*4 + 3] * f3;

    pfres2[2*4 + 2] =   pf[3*4 + 0] * f1 -  pf[3*4 + 1] * f2 + pf[3*4 + 3] * fd0;
    pfres2[2*4 + 3] = -(pf[2*4 + 0] * f1 -  pf[2*4 + 1] * f2 + pf[2*4 + 3] * fd0);

    pfres2[3*4 + 2] = -(pf[3*4 + 0] * fd1 - pf[3*4 + 1] * f3 + pf[3*4 + 2] * fd0);
    pfres2[3*4 + 3] =   pf[2*4 + 0] * fd1 - pf[2*4 + 1] * f3 + pf[2*4 + 2] * fd0;

    T fdet = pf[0*4 + 3] * pfres2[3*4 + 0] + pf[1*4 + 3] * pfres2[3*4 + 1] +
             pf[2*4 + 3] * pfres2[3*4 + 2] + pf[3*4 + 3] * pfres2[3*4 + 3];

    if( fabs(fdet) < 1e-9) return NULL;

    fdet = 1 / fdet;
    //if( pfdet != NULL ) *pfdet = fdet;

    if( pfres2 == pfres ) {
        mult(pfres, fdet, 16);
        return pfres;
    }

    int i = 0;
    while(i < 16) {
        pfres[i] = pfres2[i] * fdet;
        ++i;
    }

    return pfres;
}

/// \brief Transpose a 3x3 matrix.
template <typename T>
inline T* _transpose3(const T* pf, T* pfres)
{
    MATH_ASSERT( pf != NULL && pfres != NULL );

    if( pf == pfres ) {
        std::swap(pfres[1], pfres[3]);
        std::swap(pfres[2], pfres[6]);
        std::swap(pfres[5], pfres[7]);
        return pfres;
    }

    pfres[0] = pf[0];       pfres[1] = pf[3];       pfres[2] = pf[6];
    pfres[3] = pf[1];       pfres[4] = pf[4];       pfres[5] = pf[7];
    pfres[6] = pf[2];       pfres[7] = pf[5];       pfres[8] = pf[8];

    return pfres;
}

/// \brief Transpose a 4x4 matrix.
template <typename T>
inline T* _transpose4(const T* pf, T* pfres)
{
    MATH_ASSERT( pf != NULL && pfres != NULL );

    if( pf == pfres ) {
        std::swap(pfres[1], pfres[4]);
        std::swap(pfres[2], pfres[8]);
        std::swap(pfres[3], pfres[12]);
        std::swap(pfres[6], pfres[9]);
        std::swap(pfres[7], pfres[13]);
        std::swap(pfres[11], pfres[15]);
        return pfres;
    }

    pfres[0] = pf[0];       pfres[1] = pf[4];       pfres[2] = pf[8];               pfres[3] = pf[12];
    pfres[4] = pf[1];       pfres[5] = pf[5];       pfres[6] = pf[9];               pfres[7] = pf[13];
    pfres[8] = pf[2];       pfres[9] = pf[6];       pfres[10] = pf[10];             pfres[11] = pf[14];
    pfres[12] = pf[3];      pfres[13] = pf[7];      pfres[14] = pf[11];             pfres[15] = pf[15];
    return pfres;
}

template <typename T>
inline T _dot2(const T* pf1, const T* pf2)
{
    MATH_ASSERT( pf1 != NULL && pf2 != NULL );
    return pf1[0]*pf2[0] + pf1[1]*pf2[1];
}

template <typename T>
inline T _dot3(const T* pf1, const T* pf2)
{
    MATH_ASSERT( pf1 != NULL && pf2 != NULL );
    return pf1[0]*pf2[0] + pf1[1]*pf2[1] + pf1[2]*pf2[2];
}

template <typename T>
inline T _dot4(const T* pf1, const T* pf2)
{
    MATH_ASSERT( pf1 != NULL && pf2 != NULL );
    return pf1[0]*pf2[0] + pf1[1]*pf2[1] + pf1[2]*pf2[2] + pf1[3] * pf2[3];
}

template <typename T>
inline T _lengthsqr2(const T* pf)
{
    MATH_ASSERT( pf != NULL );
    return pf[0] * pf[0] + pf[1] * pf[1];
}

template <typename T>
inline T _lengthsqr3(const T* pf)
{
    MATH_ASSERT( pf != NULL );
    return pf[0] * pf[0] + pf[1] * pf[1] + pf[2] * pf[2];
}

template <typename T>
inline T _lengthsqr4(const T* pf)
{
    MATH_ASSERT( pf != NULL );
    return pf[0] * pf[0] + pf[1] * pf[1] + pf[2] * pf[2] + pf[3] * pf[3];
}

template <typename T>
inline T* _normalize2(T* pfout, const T* pf)
{
    MATH_ASSERT(pf != NULL);

    T f = pf[0]*pf[0] + pf[1]*pf[1];
    f = 1 / sqrt(f);
    pfout[0] = pf[0] * f;
    pfout[1] = pf[1] * f;

    return pfout;
}

template <typename T>
inline T* _normalize3(T* pfout, const T* pf)
{
    MATH_ASSERT(pf != NULL);

    T f = pf[0]*pf[0] + pf[1]*pf[1] + pf[2]*pf[2];

    f = 1 / sqrt(f);
    pfout[0] = pf[0] * f;
    pfout[1] = pf[1] * f;
    pfout[2] = pf[2] * f;

    return pfout;
}

template <typename T>
inline T* _normalize4(T* pfout, const T* pf)
{
    MATH_ASSERT(pf != NULL);

    T f = pf[0]*pf[0] + pf[1]*pf[1] + pf[2]*pf[2] + pf[3]*pf[3];

    f = 1 / sqrt(f);
    pfout[0] = pf[0] * f;
    pfout[1] = pf[1] * f;
    pfout[2] = pf[2] * f;
    pfout[3] = pf[3] * f;

    return pfout;
}

template <typename T>
inline T* _cross3(T* pfout, const T* pf1, const T* pf2)
{
    MATH_ASSERT( pfout != NULL && pf1 != NULL && pf2 != NULL );

    T temp[3];
    temp[0] = pf1[1] * pf2[2] - pf1[2] * pf2[1];
    temp[1] = pf1[2] * pf2[0] - pf1[0] * pf2[2];
    temp[2] = pf1[0] * pf2[1] - pf1[1] * pf2[0];

    pfout[0] = temp[0]; pfout[1] = temp[1]; pfout[2] = temp[2];
    return pfout;
}

template <typename T>
inline T* _transnorm3(T* pfout, const T* pfmat, const T* pf)
{
    MATH_ASSERT( pfout != NULL && pf != NULL && pfmat != NULL );

    T dummy[3];
    T* pfreal = (pfout == pf) ? dummy : pfout;

    pfreal[0] = pf[0] * pfmat[0] + pf[1] * pfmat[1] + pf[2] * pfmat[2];
    pfreal[1] = pf[0] * pfmat[3] + pf[1] * pfmat[4] + pf[2] * pfmat[5];
    pfreal[2] = pf[0] * pfmat[6] + pf[1] * pfmat[7] + pf[2] * pfmat[8];

    if( pfout ==pf ) {
        pfout[0] = pfreal[0];
        pfout[1] = pfreal[1];
        pfout[2] = pfreal[2];
    }

    return pfout;
}

inline float* mult4(float* pfres, const float* pf1, const float* pf2) {
    return _mult4<float>(pfres, pf1, pf2);
}
// pf1^T * pf2
inline float* multtrans3(float* pfres, const float* pf1, const float* pf2) {
    return _multtrans3<float>(pfres, pf1, pf2);
}
inline float* multtrans4(float* pfres, const float* pf1, const float* pf2) {
    return _multtrans4<float>(pfres, pf1, pf2);
}
inline float* transnorm3(float* pfout, const float* pfmat, const float* pf) {
    return _transnorm3<float>(pfout, pfmat, pf);
}

inline float* transpose3(const float* pf, float* pfres) {
    return _transpose3<float>(pf, pfres);
}
inline float* transpose4(const float* pf, float* pfres) {
    return _transpose4<float>(pf, pfres);
}

inline float dot2(const float* pf1, const float* pf2) {
    return _dot2<float>(pf1, pf2);
}
inline float dot3(const float* pf1, const float* pf2) {
    return _dot3<float>(pf1, pf2);
}
inline float dot4(const float* pf1, const float* pf2) {
    return _dot4<float>(pf1, pf2);
}

inline float lengthsqr2(const float* pf) {
    return _lengthsqr2<float>(pf);
}
inline float lengthsqr3(const float* pf) {
    return _lengthsqr3<float>(pf);
}
inline float lengthsqr4(const float* pf) {
    return _lengthsqr4<float>(pf);
}

inline float* normalize2(float* pfout, const float* pf) {
    return _normalize2<float>(pfout, pf);
}
inline float* normalize3(float* pfout, const float* pf) {
    return _normalize3<float>(pfout, pf);
}
inline float* normalize4(float* pfout, const float* pf) {
    return _normalize4<float>(pfout, pf);
}

inline float* cross3(float* pfout, const float* pf1, const float* pf2) {
    return _cross3<float>(pfout, pf1, pf2);
}

// multiplies 3x3 matrices
inline float* mult3_s4(float* pfres, const float* pf1, const float* pf2) {
    return _mult3_s4<float>(pfres, pf1, pf2);
}
inline float* mult3_s3(float* pfres, const float* pf1, const float* pf2) {
    return _mult3_s3<float>(pfres, pf1, pf2);
}

inline float* inv3(const float* pf, float* pfres, float* pfdet, int stride) {
    return _inv3<float>(pf, pfres, pfdet, stride);
}
inline float* inv4(const float* pf, float* pfres) {
    return _inv4<float>(pf, pfres);
}


inline double* mult4(double* pfres, const double* pf1, const double* pf2) {
    return _mult4<double>(pfres, pf1, pf2);
}
// pf1^T * pf2
inline double* multtrans3(double* pfres, const double* pf1, const double* pf2) {
    return _multtrans3<double>(pfres, pf1, pf2);
}
inline double* multtrans4(double* pfres, const double* pf1, const double* pf2) {
    return _multtrans4<double>(pfres, pf1, pf2);
}
inline double* transnorm3(double* pfout, const double* pfmat, const double* pf) {
    return _transnorm3<double>(pfout, pfmat, pf);
}

inline double* transpose3(const double* pf, double* pfres) {
    return _transpose3<double>(pf, pfres);
}
inline double* transpose4(const double* pf, double* pfres) {
    return _transpose4<double>(pf, pfres);
}

inline double dot2(const double* pf1, const double* pf2) {
    return _dot2<double>(pf1, pf2);
}
inline double dot3(const double* pf1, const double* pf2) {
    return _dot3<double>(pf1, pf2);
}
inline double dot4(const double* pf1, const double* pf2) {
    return _dot4<double>(pf1, pf2);
}

inline double lengthsqr2(const double* pf) {
    return _lengthsqr2<double>(pf);
}
inline double lengthsqr3(const double* pf) {
    return _lengthsqr3<double>(pf);
}
inline double lengthsqr4(const double* pf) {
    return _lengthsqr4<double>(pf);
}

inline double* normalize2(double* pfout, const double* pf) {
    return _normalize2<double>(pfout, pf);
}
inline double* normalize3(double* pfout, const double* pf) {
    return _normalize3<double>(pfout, pf);
}
inline double* normalize4(double* pfout, const double* pf) {
    return _normalize4<double>(pfout, pf);
}

inline double* cross3(double* pfout, const double* pf1, const double* pf2) {
    return _cross3<double>(pfout, pf1, pf2);
}

// multiplies 3x3 matrices
inline double* mult3_s4(double* pfres, const double* pf1, const double* pf2) {
    return _mult3_s4<double>(pfres, pf1, pf2);
}
inline double* mult3_s3(double* pfres, const double* pf1, const double* pf2) {
    return _mult3_s3<double>(pfres, pf1, pf2);
}

inline double* inv3(const double* pf, double* pfres, double* pfdet, int stride) {
    return _inv3<double>(pf, pfres, pfdet, stride);
}
inline double* inv4(const double* pf, double* pfres) {
    return _inv4<double>(pf, pfres);
}

template <typename T, typename R, typename S>
inline S* mult(T* pf1, R* pf2, int r1, int c1, int c2, S* pfres, bool badd)
{
    MATH_ASSERT( pf1 != NULL && pf2 != NULL && pfres != NULL);
    int j, k;

    if( !badd ) memset(pfres, 0, sizeof(S) * r1 * c2);

    while(r1 > 0) {
        --r1;

        j = 0;
        while(j < c2) {
            k = 0;
            while(k < c1) {
                pfres[j] += (S)(pf1[k] * pf2[k*c2 + j]);
                ++k;
            }
            ++j;
        }

        pf1 += c1;
        pfres += c2;
    }

    return pfres;
}

template <typename T, typename R, typename S>
inline S* multtrans(T* pf1, R* pf2, int r1, int c1, int c2, S* pfres, bool badd)
{
    MATH_ASSERT( pf1 != NULL && pf2 != NULL && pfres != NULL);
    int i, j, k;

    if( !badd ) memset(pfres, 0, sizeof(S) * c1 * c2);

    i = 0;
    while(i < c1) {

        j = 0;
        while(j < c2) {

            k = 0;
            while(k < r1) {
                pfres[j] += (S)(pf1[k*c1] * pf2[k*c2 + j]);
                ++k;
            }
            ++j;
        }

        pfres += c2;
        ++pf1;

        ++i;
    }

    return pfres;
}

template <typename T, typename R, typename S>
inline S* multtrans_to2(T* pf1, R* pf2, int r1, int c1, int r2, S* pfres, bool badd)
{
    MATH_ASSERT( pf1 != NULL && pf2 != NULL && pfres != NULL);
    int j, k;

    if( !badd ) memset(pfres, 0, sizeof(S) * r1 * r2);

    while(r1 > 0) {
        --r1;

        j = 0;
        while(j < r2) {
            k = 0;
            while(k < c1) {
                pfres[j] += (S)(pf1[k] * pf2[j*c1 + k]);
                ++k;
            }
            ++j;
        }

        pf1 += c1;
        pfres += r2;
    }

    return pfres;
}

template <typename T> inline T* multto1(T* pf1, T* pf2, int r, int c, T* pftemp)
{
    MATH_ASSERT( pf1 != NULL && pf2 != NULL );

    int j, k;
    bool bdel = false;

    if( pftemp == NULL ) {
        pftemp = new T[c];
        bdel = true;
    }

    while(r > 0) {
        --r;

        j = 0;
        while(j < c) {

            pftemp[j] = 0.0;

            k = 0;
            while(k < c) {
                pftemp[j] += pf1[k] * pf2[k*c + j];
                ++k;
            }
            ++j;
        }

        memcpy(pf1, pftemp, c * sizeof(T));
        pf1 += c;
    }

    if( bdel ) delete[] pftemp;

    return pf1;
}

template <typename T, typename S> inline T* multto2(T* pf1, S* pf2, int r2, int c2, S* pftemp)
{
    MATH_ASSERT( pf1 != NULL && pf2 != NULL );

    int i, j, k;
    bool bdel = false;

    if( pftemp == NULL ) {
        pftemp = new S[r2];
        bdel = true;
    }

    // do columns first
    j = 0;
    while(j < c2) {
        i = 0;
        while(i < r2) {

            pftemp[i] = 0.0;

            k = 0;
            while(k < r2) {
                pftemp[i] += pf1[i*r2 + k] * pf2[k*c2 + j];
                ++k;
            }
            ++i;
        }

        i = 0;
        while(i < r2) {
            *(pf2+i*c2+j) = pftemp[i];
            ++i;
        }

        ++j;
    }

    if( bdel ) delete[] pftemp;

    return pf1;
}

template <typename T> inline void add(T* pf1, T* pf2, int r)
{
    MATH_ASSERT( pf1 != NULL && pf2 != NULL);

    while(r > 0) {
        --r;
        pf1[r] += pf2[r];
    }
}

template <typename T> inline void sub(T* pf1, T* pf2, int r)
{
    MATH_ASSERT( pf1 != NULL && pf2 != NULL);

    while(r > 0) {
        --r;
        pf1[r] -= pf2[r];
    }
}

template <typename T> inline T normsqr(const T* pf1, int r)
{
    MATH_ASSERT( pf1 != NULL );

    T d = 0.0;
    while(r > 0) {
        --r;
        d += pf1[r] * pf1[r];
    }

    return d;
}

template <typename T> inline T lengthsqr(const T* pf1, const T* pf2, int length)
{
    T d = 0;
    while(length > 0) {
        --length;
        T t = pf1[length] - pf2[length];
        d += t * t;
    }

    return d;
}

template <typename T> inline T dot(T* pf1, T* pf2, int length)
{
    T d = 0;
    while(length > 0) {
        --length;
        d += pf1[length] * pf2[length];
    }

    return d;
}

template <typename T> inline T sum(T* pf, int length)
{
    T d = 0;
    while(length > 0) {
        --length;
        d += pf[length];
    }

    return d;
}

template <typename T> inline bool inv2(T* pf, T* pfres)
{
    T fdet = pf[0] * pf[3] - pf[1] * pf[2];

    if( fabs(fdet) < 1e-16 ) return false;

    fdet = 1 / fdet;
    //if( pfdet != NULL ) *pfdet = fdet;

    if( pfres != pf ) {
        pfres[0] = fdet * pf[3];                pfres[1] = -fdet * pf[1];
        pfres[2] = -fdet * pf[2];               pfres[3] = fdet * pf[0];
        return true;
    }

    T ftemp = pf[0];
    pfres[0] = pf[3] * fdet;
    pfres[1] *= -fdet;
    pfres[2] *= -fdet;
    pfres[3] = ftemp * pf[0];

    return true;
}

template <typename T, typename S>
void Tridiagonal3 (S* mat, T* diag, T* subd)
{
    T a, b, c, d, e, f, ell, q;

    a = mat[0*3+0];
    b = mat[0*3+1];
    c = mat[0*3+2];
    d = mat[1*3+1];
    e = mat[1*3+2];
    f = mat[2*3+2];

    subd[2] = 0.0;
    diag[0] = a;
    if ( fabs(c) >= g_fEpsilon ) {
        ell = (T)sqrt(b*b+c*c);
        b /= ell;
        c /= ell;
        q = 2*b*e+c*(f-d);
        diag[1] = d+c*q;
        diag[2] = f-c*q;
        subd[0] = ell;
        subd[1] = e-b*q;
        mat[0*3+0] = (S)1; mat[0*3+1] = (S)0; mat[0*3+2] = (T)0;
        mat[1*3+0] = (S)0; mat[1*3+1] = b; mat[1*3+2] = c;
        mat[2*3+0] = (S)0; mat[2*3+1] = c; mat[2*3+2] = -b;
    }
    else {
        diag[1] = d;
        diag[2] = f;
        subd[0] = b;
        subd[1] = e;
        mat[0*3+0] = (S)1; mat[0*3+1] = (S)0; mat[0*3+2] = (S)0;
        mat[1*3+0] = (S)0; mat[1*3+1] = (S)1; mat[1*3+2] = (S)0;
        mat[2*3+0] = (S)0; mat[2*3+1] = (S)0; mat[2*3+2] = (S)1;
    }
}

template <typename T>
inline void svd3(const T* A, T* U, T* D, T* V)
{
    T VVt[9];
    T eigenvalues[3];
    multtrans3(VVt, A, A);
    // get eigen values of V: VVt  V = V  D^2
    T afSubDiag[3];
    std::copy(&VVt,&VVt[9],&V[0]);
    Tridiagonal3(V,eigenvalues,afSubDiag);
    QLAlgorithm3(V,eigenvalues,afSubDiag);

    //float fDet =	V[0*3+0] * (V[1*3+1] * V[2*3+2] - V[1*3+2] * V[2*3+1]) +
    //        V[0*3+1] * (V[1*3+2] * V[2*3+0] - V[1*3+0] * V[2*3+2]) +
    //        V[0*3+2] * (V[1*3+0] * V[2*3+1] - V[1*3+1] * V[2*3+0]);
    //
    //    if ( fDet < 0.0f ) {
    //        V[0*3+2] = - V[0*3+2];
    //        V[1*3+2] = - V[1*3+2];
    //        V[2*3+2] = - V[2*3+2];
    //    }

    mult3_s3(U, A, V); // U = A V = U D
    for(int i = 0; i < 3; ++i) {
        D[i] = sqrt(eigenvalues[i]);
        T f = 1/D[i];
        U[i] *= f;
        U[i+3] *= f;
        U[i+6] *= f;
    }
    int maxval = 0;
    if( D[1] > D[maxval] ) {
        maxval = 1;
    }
    if( D[2] > D[maxval] ) {
        maxval = 2;
    }
    if( maxval > 0 ) {
        // flip columns
        swap(U[0], U[maxval]);
        swap(U[3], U[3+maxval]);
        swap(U[6], U[6+maxval]);
        swap(V[0], V[maxval]);
        swap(V[3], V[3+maxval]);
        swap(V[6], V[6+maxval]);
        swap(D[0], D[maxval]);
    }
    if( D[1] < D[2] ) {
        swap(U[1], U[2]);
        swap(U[4], U[5]);
        swap(U[7], U[8]);
        swap(V[1], V[2]);
        swap(V[4], V[5]);
        swap(V[7], V[8]);
        swap(D[1], D[2]);
    }
}

template <typename T>
int Min(T* pts, int stride, int numPts)
{
    MATH_ASSERT( pts != NULL && numPts > 0 && stride > 0 );

    int best = pts[0]; pts += stride;
    for(int i = 1; i < numPts; ++i, pts += stride) {
        if( best > pts[0] )
            best = pts[0];
    }

    return best;
}

template <typename T>
int Max(T* pts, int stride, int numPts)
{
    MATH_ASSERT( pts != NULL && numPts > 0 && stride > 0 );

    int best = pts[0]; pts += stride;
    for(int i = 1; i < numPts; ++i, pts += stride) {
        if( best < pts[0] )
            best = pts[0];
    }

    return best;
}

} // end namespace mathextra
} // end namespace OpenRAVE

#endif
