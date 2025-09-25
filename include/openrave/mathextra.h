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
OPENRAVE_API bool eig2(const float* pfmat, float* peigs, float& fv1x, float& fv1y, float& fv2x, float& fv2y);
OPENRAVE_API bool eig2(const double* pfmat, double* peigs, double& fv1x, double& fv1y, double& fv2x, double& fv2y);

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
    if(c == 0) { // x*(ax+b) = 0
        r1 = 0;
        r2 = -b/a;
        return 2;
    }

    T d = b * b - (T)4 * c * a;
    if( d < 0 ) {
        if( d < -(T)1e-16) {
            return 0;
        }
        // d is close to 0, so most likely floating precision error
        d = 0;
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
    std::copy(&VVt[0],&VVt[9],&V[0]);
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
        if( eigenvalues[i] > 0 ) {
            D[i] = sqrt(eigenvalues[i]);
        }
        else {
            if( eigenvalues[i] < -std::numeric_limits<T>::epsilon() ) {
                throw std::runtime_error("eigenvalue should be non-negative");
            }
            D[i] = 0; // close to 0
        }
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
        std::swap(U[0], U[maxval]);
        std::swap(U[3], U[3+maxval]);
        std::swap(U[6], U[6+maxval]);
        std::swap(V[0], V[maxval]);
        std::swap(V[3], V[3+maxval]);
        std::swap(V[6], V[6+maxval]);
        std::swap(D[0], D[maxval]);
    }
    if( D[1] < D[2] ) {
        std::swap(U[1], U[2]);
        std::swap(U[4], U[5]);
        std::swap(U[7], U[8]);
        std::swap(V[1], V[2]);
        std::swap(V[4], V[5]);
        std::swap(V[7], V[8]);
        std::swap(D[1], D[2]);
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

/// \brief Durand-Kerner polynomial root finding method
template <typename IKReal, int D>
inline void polyroots(const IKReal* rawcoeffs, IKReal* rawroots, int& numroots)
{
    using std::complex;
    static_assert(D > 1, "D should be greater than 1. Solve linear equation to handle D==1 case");  // this function accesses roots[1]

    BOOST_ASSERT(rawcoeffs[0] != 0);
    const IKReal tol = 128.0*std::numeric_limits<IKReal>::epsilon();
    const IKReal tolsqrt = sqrt(std::numeric_limits<IKReal>::epsilon());
    complex<IKReal> coeffs[D];
    const int maxsteps = 110;
    for(int i = 0; i < D; ++i) {
        coeffs[i] = complex<IKReal>(rawcoeffs[i+1]/rawcoeffs[0]);
    }
    complex<IKReal> roots[D];
    IKReal err[D];
    roots[0] = complex<IKReal>(1,0);
    roots[1] = complex<IKReal>(0.4,0.9); // any complex number not a root of unity works
    err[0] = 1.0;
    err[1] = 1.0;
    for(int i = 2; i < D; ++i) {
        roots[i] = roots[i-1]*roots[1];
        err[i] = 1.0;
    }
    for(int step = 0; step < maxsteps; ++step) {
        bool changed = false;
        for(int i = 0; i < D; ++i) {
            if ( err[i] >= tol ) {
                changed = true;
                // evaluate
                complex<IKReal> x = roots[i] + coeffs[0];
                for(int j = 1; j < D; ++j) {
                    x = roots[i] * x + coeffs[j];
                }
                for(int j = 0; j < D; ++j) {
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
    bool visited[D] = {false};
    for(int i = 0; i < D; ++i) {
        if( !visited[i] ) {
            // might be a multiple root, in which case it will have more error than the other roots
            // find any neighboring roots, and take the average
            complex<IKReal> newroot=roots[i];
            int n = 1;
            for(int j = i+1; j < D; ++j) {
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
            if( RaveFabs(imag(newroot)) < tolsqrt ) {
                rawroots[numroots++] = real(newroot);
            }
        }
    }
}

// Polynomial utilities

// Coefficient computation given boundary conditions

// Given boundary conditions (x0, dx0, ddx0), (x1, dx1, ddx1), and duration t, compute the
// corresponding quintic coefficients a, b, c, d, e, f:
// p(x) = ax^5 + bx^4 + cx^3 + dx^2 + ex + f.
template <typename T>
inline void computequinticcoeffs(T x0, T x1, T dx0, T dx1, T ddx0, T ddx1, T t, T* coeffs)
{
    T t2 = t*t;
    T t3 = t2*t;
    T t4 = t3*t;
    T t5 = t4*t;

    coeffs[0] = (t2*(ddx1 - ddx0) - 6.0*t*(dx1 + dx0) + 12.0*(x1 - x0))/(2*t5);
    coeffs[1] = (t2*(3.0*ddx0 - 2.0*ddx1) + t*(16.0*dx0 + 14.0*dx1) + 30.0*(x0 - x1))/(2*t4);
    coeffs[2] = (t2*(ddx1 - 3.0*ddx0) - t*(12.0*dx0 + 8.0*dx1) + 20.0*(x1 - x0))/(2*t3);
    coeffs[3] = 0.5*ddx0;
    coeffs[4] = dx0;
    coeffs[5] = x0;
    return;
}

// Given boundary conditions (x0, dx0, ddx0), (x1, dx1, ddx1), and duration t, compute the
// corresponding cubic coefficients a, b, c, d:
// p(x) = ax^3 + bx^2 + cx + d.
// This function assumes that the given boundary conditions are consistent.
template <typename T>
inline void computecubiccoeffs(T x0, T x1, T dx0, T dx1, T ddx0, T ddx1, T t, T* coeffs)
{
    coeffs[3] = x0;
    coeffs[2] = dx0;
    coeffs[1] = 0.5*ddx0;

    // T candidate1 = (x1 - x0 - dx0*t - 0.5*ddx0*t2)/t3;
    // T candidate2 = 2*(x0 - x1)/t3 + (dx0 + dx1)/t2;
    // T candidate3 = (ddx1 - ddx0)/(6*t);

    // Use the following formula to evaluate the first coefficient since it is
    // using acceleration terms. The accleration terms themselves were evaluated
    // from the expression 6*a*t + 2*b, which only uses a and b and therefore
    // least affected by numerical errors from the zero-th and first derivatives.
    coeffs[0] = (ddx1 - ddx0)/(6*t);
    return;
}

// Evaluation functions

// Evaluate the given quintic at the given point. Strongest coefficient first.
template <typename T>
inline void evaluatequintic(const T* coeffs, const T t, T& val)
{
    val = coeffs[5] + t*(coeffs[4] + t*(coeffs[3] + t*(coeffs[2] + t*(coeffs[1] + t*coeffs[0]))));
}

// Evaluate the first derivative of the given quintic at the given point. Strongest coefficient first.
template <typename T>
inline void evaluatequinticderiv1(const T* coeffs, const T t, T& val)
{
    val = coeffs[4] + t*(2*coeffs[3] + t*(3*coeffs[2] + t*(4*coeffs[1] + t*5*coeffs[0])));
}

// Evaluate the second derivative of the given quintic at the given point. Strongest coefficient first.
template <typename T>
inline void evaluatequinticderiv2(const T* coeffs, const T t, T& val)
{
    val = 2*coeffs[3] + t*(6*coeffs[2] + t*(12*coeffs[1] + t*20*coeffs[0]));
}

// Evaluate the given cubic at the given point. Strongest coefficient first.
template <typename T>
inline void evaluatecubic(const T* coeffs, const T t, T& val)
{
    val = coeffs[3] + t*(coeffs[2] + t*(coeffs[1] + t*(coeffs[0])));
}

// Evaluate the first derivative of the given cubic at the given point. Strongest coefficient first.
template <typename T>
inline void evaluatecubicderiv1(const T* coeffs, const T t, T& val)
{
    val = coeffs[2] + t*(2*coeffs[1] + t*(3*coeffs[0]));
}

// Evaluate the second derivative of the given cubic at the given point. Strongest coefficient first.
template <typename T>
inline void evaluatecubicderiv2(const T* coeffs, const T t, T& val)
{
    val = 2*coeffs[1] + t*(6*coeffs[0]);
}

// Functions for computing critical points.

// Given a set of quintic coefficients, find all critical points (points at which the first
// derivative vanishes).
template <typename T>
inline void computequinticcriticalpoints(const T* coeffs, T* criticalpoints, int& numpoints)
{
    const T quarticcoeffs[] = {5*coeffs[0], 4*coeffs[1], 3*coeffs[2], 2*coeffs[3], coeffs[4]};
    polyroots<T, 4>(quarticcoeffs, criticalpoints, numpoints);
}

// Given a set of quartic coefficients, find all critical points (points at which the first
// derivative vanishes).
template <typename T>
inline void computequarticcriticalpoints(const T* coeffs, T* criticalpoints, int& numpoints)
{
    const T cubiccoeffs[] = {4*coeffs[0], 3*coeffs[1], 2*coeffs[2], coeffs[3]};
    polyroots<T, 3>(cubiccoeffs, criticalpoints, numpoints);
}

// Given a set of cubic coefficients, find all critical points (points at which the first
// derivative vanishes).
template <typename T>
inline void computecubiccriticalpoints(const T* coeffs, T* criticalpoints, int& numpoints)
{
    const T quadraticcoeffs[] = {3*coeffs[0], 2*coeffs[1], coeffs[2]};
    polyroots<T, 2>(quadraticcoeffs, criticalpoints, numpoints);
}

// Given a set of quadratic coefficients, find all critical points (points at which the first
// derivative vanishes).
template <typename T>
inline void computequadraticcriticalpoints(const T* coeffs, T* criticalpoints, int& numpoints)
{
    // const T linearcoeffs[] = {2*coeffs[0], coeffs[1]};
    BOOST_ASSERT(coeffs[0] != 0);
    numpoints = 1;
    criticalpoints[0] = -0.5*coeffs[1]/coeffs[0];
}

// Given a set of coefficients of a quintic p(t) (strongest coefficient first), find smallest tdelta
// > 0 such that p(t + tdelta) = p(tcur) + step. (step can be negative.)  *** Assume that if there
// exists a critical point tc to the right of tcur, |p(tc) - p(tcur)| >= |step|. This is to
// guarantee that tnext always exists.
template <typename T>
inline bool computequinticnextdiscretizedstep(const T* coeffs, const T step, const T tcur, T& tdelta)
{
    // Evaluate the 0th, 1st, 2nd, ..., 5th derivatives of p
    // T p = coeffs[5] + tcur*(coeffs[4] + tcur*(coeffs[3] + tcur*(coeffs[2] + tcur*(coeffs[1] + tcur*coeffs[0]))));
    T tempcoeffs[] = {5*coeffs[0], 4*coeffs[1], 3*coeffs[2], 2*coeffs[3], coeffs[4], 0};
    T d1p = tempcoeffs[4] + tcur*(tempcoeffs[3] + tcur*(tempcoeffs[2] + tcur*(tempcoeffs[1] + tcur*tempcoeffs[0])));

    tempcoeffs[0] *= 4; tempcoeffs[1] *= 3; tempcoeffs[2] *= 2;
    T d2p = tempcoeffs[3] + tcur*(tempcoeffs[2] + tcur*(tempcoeffs[1] + tcur*tempcoeffs[0]));

    tempcoeffs[0] *= 3; tempcoeffs[1] *= 2;
    T d3p = tempcoeffs[2] + tcur*(tempcoeffs[1] + tcur*tempcoeffs[0]);

    tempcoeffs[0] *= 2;
    T d4p = tempcoeffs[1] + tcur*tempcoeffs[0];

    T d5p = tempcoeffs[0];

    // tempcoeffs holds coefficients of a polynomial resulting from Taylor series expansion of p at t = tcur.
    tempcoeffs[0] = d5p/120;
    tempcoeffs[1] = d4p/24;
    tempcoeffs[2] = d3p/6;
    tempcoeffs[3] = d2p/2;
    tempcoeffs[4] = d1p;
    tempcoeffs[5] = -step;

    int numroots = 0;
    T rawroots[5];
    if( tempcoeffs[0] != 0 ) {
        polyroots<T, 5>(tempcoeffs, rawroots, numroots);
    }
    else if( tempcoeffs[1] != 0 ) {
        polyroots<T, 4>(&tempcoeffs[1], rawroots, numroots);
    }
    else if( tempcoeffs[2] != 0 ) {
        polyroots<T, 3>(&tempcoeffs[2], rawroots, numroots);
    }
    else if( tempcoeffs[3] != 0 ) {
        polyroots<T, 2>(&tempcoeffs[3], rawroots, numroots);
    }
    else if( tempcoeffs[4] != 0 ) {
        // polyroots<T, 1>(&tempcoeffs[4], rawroots, numroots);  // out-of-bound memory access
        numroots = 1;
        rawroots[0] = -tempcoeffs[5]/tempcoeffs[4];
    }
    bool bFound = false;
    for( int i = 0; i < numroots; ++i ) {
        if( rawroots[i] > 0 ) {
            if( !bFound || (rawroots[i] < tdelta) ) {
                bFound = true;
                tdelta = rawroots[i];
            }
        }
    }
    return bFound;
}

// Given a set of coefficients of a cubic p(t) (strongest coefficient first), find smallest tdelta
// > 0 such that p(tcur + tdelta) = p(tcur) + step. (step can be negative.)  *** Assume that if there
// exists a critical point tc to the right of tcur, |p(tc) - p(tcur)| >= |step|. This is to
// guarantee that tnext always exists.
template <typename T>
inline bool computecubicnextdiscretizedstep(const T* coeffs, const T step, const T tcur, T& tdelta)
{
    // Evaluate the 0th, 1st, 2nd, and 3rd derivatives of p
    // T p = coeffs[3] + tcur*(coeffs[2] + tcur*(coeffs[1] + tcur*(coeffs[0])));
    T tempcoeffs[] = {3*coeffs[0], 2*coeffs[1], coeffs[2], 0};
    T d1p = tempcoeffs[2] + tcur*(tempcoeffs[1] + tcur*(tempcoeffs[0]));

    tempcoeffs[0] *= 2;
    T d2p = tempcoeffs[1] + tcur*(tempcoeffs[0]);

    T d3p = tempcoeffs[0];

    // tempcoeffs holds coefficients of a polynomial resulting from Taylor series expansion of p at t = tcur.
    tempcoeffs[0] = d3p/6;
    tempcoeffs[1] = d2p/2;
    tempcoeffs[2] = d1p;
    tempcoeffs[3] = -step;

    int numroots = 0;
    T rawroots[3];
    if( tempcoeffs[0] != 0 ) {
        polyroots<T, 3>(tempcoeffs, rawroots, numroots);
    }
    else if( tempcoeffs[1] != 0 ) {
        polyroots<T, 2>(&tempcoeffs[1], rawroots, numroots);
    }
    else if( tempcoeffs[2] != 0 ) {
        // polyroots<T, 1>(&tempcoeffs[2], rawroots, numroots);  // out-of-bound memory access
        numroots = 1;
        rawroots[0] = -tempcoeffs[3]/tempcoeffs[2];
    }
    bool bFound = false;
    for( int i = 0; i < numroots; ++i ) {
        if( rawroots[i] > 0 ) {
            if( !bFound || (rawroots[i] < tdelta) ) {
                bFound = true;
                tdelta = rawroots[i];
            }
        }
    }
    return bFound;
}

} // end namespace mathextra
} // end namespace OpenRAVE

#endif
