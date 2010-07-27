// Copyright (C) 2006-2010 Rosen Diankov (rosen.diankov@gmail.com)
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
/** \file   geometry.h
    \brief  Basic gemoetric primitives and affine math functions on them.
 */
#ifndef OPENRAVE_GEOMETRY_H
#define OPENRAVE_GEOMETRY_H

#include <cmath>
#include <iostream>
#include <cstring>
#include <climits>
#include <cstdlib>

#ifdef BOOST_ASSERT
#define MATH_ASSERT BOOST_ASSERT
#else
#include <cassert>
#define MATH_ASSERT assert
#endif

#ifndef g_fEpsilon
#define g_fEpsilon 2e-7f
#endif

#ifndef PI
#define PI 3.14159265358979323846
#endif

namespace OpenRAVE {

/// Templated math and geometric functions
namespace geometry {

template <typename T> class RaveTransform;
template <typename T> class RaveTransformMatrix;

inline float RaveSqrt(float f) { return sqrtf(f); }
inline double RaveSqrt(double f) { return sqrt(f); }
inline float RaveSin(float f) { return sinf(f); }
inline double RaveSin(double f) { return sin(f); }
inline float RaveCos(float f) { return cosf(f); }
inline double RaveCos(double f) { return cos(f); }
inline float RaveFabs(float f) { return fabsf(f); }
inline double RaveFabs(double f) { return fabs(f); }
inline float RaveAcos(float f) { return acosf(f); }
inline double RaveAcos(double f) { return acos(f); }
inline float RaveAsin(float f) { return asinf(f); }
inline double RaveAsin(double f) { return asin(f); }
inline float RaveAtan2(float fy, float fx) { return atan2f(fy,fx); }
inline double RaveAtan2(double fy, double fx) { return atan2(fy,fx); }

/** \brief Vector class containing 4 dimensions.
    
    \ingroup affine_math
     It is better to use this for a 3 dim vector because it is 16byte aligned and SIMD instructions can be used
*/
template <typename T>
class RaveVector
{
public:
    T x, y, z, w;
    
    RaveVector() : x(0), y(0), z(0), w(0) {}
        
    RaveVector(T x, T y, T z) : x(x), y(y), z(z), w(0) {}
    RaveVector(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {}
    template<typename U> RaveVector(const RaveVector<U> &vec) : x((T)vec.x), y((T)vec.y), z((T)vec.z), w((T)vec.w) {}

    /// note, it only copes 3 values!
    template<typename U> RaveVector(const U* pf) { MATH_ASSERT(pf != NULL); x = (T)pf[0]; y = (T)pf[1]; z = (T)pf[2]; w = 0; }
    
    T  operator[](int i) const       { return (&x)[i]; }
    T& operator[](int i)             { return (&x)[i]; }
    
    // casting operators
    operator T* () { return &x; }
    operator const T* () const { return (const T*)&x; }

    template <typename U>
    RaveVector<T>& operator=(const RaveVector<U>& r) { x = (T)r.x; y = (T)r.y; z = (T)r.z; w = (T)r.w; return *this; }
    
    // SCALAR FUNCTIONS
    template <typename U> inline T dot(const RaveVector<U> &v) const { return x*v.x + y*v.y + z*v.z + w*v.w; }
    template <typename U> inline T dot3(const RaveVector<U> &v) const { return x*v.x + y*v.y + z*v.z; }
    inline RaveVector<T>& normalize() { return normalize4(); }
    inline RaveVector<T>& normalize4() {
        T f = x*x+y*y+z*z+w*w;
        MATH_ASSERT( f > 0 );
        f = 1.0f / RaveSqrt(f);
        x *= f; y *= f; z *= f; w *= f;
        return *this;
    }
    inline RaveVector<T>& normalize3() {
        T f = x*x+y*y+z*z;
        MATH_ASSERT( f > 0 );
        f = 1.0f / RaveSqrt(f);
        x *= f; y *= f; z *= f;
        return *this;
    }

    inline T lengthsqr2() const { return x*x + y*y; }
    inline T lengthsqr3() const { return x*x + y*y + z*z; }
    inline T lengthsqr4() const { return x*x + y*y + z*z + w*w; }

    inline void Set3(const T* pvals) { x = pvals[0]; y = pvals[1]; z = pvals[2]; }
    inline void Set3(T val1, T val2, T val3) { x = val1; y = val2; z = val3; }
    inline void Set4(const T* pvals) { x = pvals[0]; y = pvals[1]; z = pvals[2]; w = pvals[3]; }
    inline void Set4(T val1, T val2, T val3, T val4) { x = val1; y = val2; z = val3; w = val4;}
    /// 3 dim cross product, w is not touched
    inline RaveVector<T> cross(const RaveVector<T> &v) const {
        RaveVector<T> ucrossv;
        ucrossv[0] = y * v[2] - z * v[1];
        ucrossv[1] = z * v[0] - x * v[2];
        ucrossv[2] = x * v[1] - y * v[0];
        return ucrossv;
    }

    inline RaveVector<T>& Cross(const RaveVector<T> &v) RAVE_DEPRECATED { Cross(*this, v); return *this; }
    inline RaveVector<T>& Cross(const RaveVector<T> &u, const RaveVector<T> &v) RAVE_DEPRECATED
    {
        RaveVector<T> ucrossv;
        ucrossv[0] = u[1] * v[2] - u[2] * v[1];
        ucrossv[1] = u[2] * v[0] - u[0] * v[2];
        ucrossv[2] = u[0] * v[1] - u[1] * v[0];
        *this = ucrossv;
        return *this;
    }

    inline RaveVector<T> operator-() const { RaveVector<T> v; v.x = -x; v.y = -y; v.z = -z; v.w = -w; return v; }
    template <typename U> inline RaveVector<T> operator+(const RaveVector<U> &r) const { RaveVector<T> v; v.x = x+r.x; v.y = y+r.y; v.z = z+r.z; v.w = w+r.w; return v; }
    template <typename U> inline RaveVector<T> operator-(const RaveVector<U> &r) const { RaveVector<T> v; v.x = x-r.x; v.y = y-r.y; v.z = z-r.z; v.w = w-r.w; return v; }
    template <typename U> inline RaveVector<T> operator*(const RaveVector<U> &r) const { RaveVector<T> v; v.x = r.x*x; v.y = r.y*y; v.z = r.z*z; v.w = r.w*w; return v; }
    inline RaveVector<T> operator*(T k) const { RaveVector<T> v; v.x = k*x; v.y = k*y; v.z = k*z; v.w = k*w; return v; }

    template <typename U> inline RaveVector<T>& operator += (const RaveVector<U>& r) { x += r.x; y += r.y; z += r.z; w += r.w; return *this; }
    template <typename U> inline RaveVector<T>& operator -= (const RaveVector<U>& r) { x -= r.x; y -= r.y; z -= r.z; w -= r.w; return *this; }
    template <typename U> inline RaveVector<T>& operator *= (const RaveVector<U>& r) { x *= r.x; y *= r.y; z *= r.z; w *= r.w; return *this; }
    
    inline RaveVector<T>& operator *= (const T k) { x *= k; y *= k; z *= k; w *= k; return *this; }
    inline RaveVector<T>& operator /= (const T _k) { T k=1/_k; x *= k; y *= k; z *= k; w *= k; return *this; }

    template <typename U> friend RaveVector<U> operator* (float f, const RaveVector<U>& v);
    template <typename U> friend RaveVector<U> operator* (double f, const RaveVector<U>& v);
    
    template <typename S, typename U> friend std::basic_ostream<S>& operator<<(std::basic_ostream<S>& O, const RaveVector<U>& v);
    template <typename S, typename U> friend std::basic_istream<S>& operator>>(std::basic_istream<S>& I, RaveVector<U>& v);

    /// cross product operator
    template <typename U> inline RaveVector<T> operator^(const RaveVector<U> &v) const { 
        RaveVector<T> ucrossv;
        ucrossv[0] = y * v[2] - z * v[1];
        ucrossv[1] = z * v[0] - x * v[2];
        ucrossv[2] = x * v[1] - y * v[0];
        return ucrossv;
    }
};

template <typename T>
inline RaveVector<T> operator* (float f, const RaveVector<T>& left)
{
    RaveVector<T> v;
    v.x = (T)f * left.x;
    v.y = (T)f * left.y;
    v.z = (T)f * left.z;
    v.w = (T)f * left.w;
    return v;
}

template <typename T>
inline RaveVector<T> operator* (double f, const RaveVector<T>& left)
{
    RaveVector<T> v;
    v.x = (T)f * left.x;
    v.y = (T)f * left.y;
    v.z = (T)f * left.z;
    v.w = (T)f * left.w;
    return v;
}

/** \brief Affine transformation parameterized with quaterions.
    
    \ingroup affine_math
*/
template <typename T>
class RaveTransform
{
public:
    RaveTransform() { rot.x = 1; }
    template <typename U> RaveTransform(const RaveTransform<U>& t) {
        rot = t.rot;
        trans = t.trans;
        T fnorm = rot.lengthsqr4();
        MATH_ASSERT( fnorm > 0.99f && fnorm < 1.01f );
    }
    template <typename U> RaveTransform(const RaveVector<U>& rot, const RaveVector<U>& trans) : rot(rot), trans(trans) {
        T fnorm = rot.lengthsqr4();
        MATH_ASSERT( fnorm > 0.99f && fnorm < 1.01f );
    }
    inline RaveTransform(const RaveTransformMatrix<T>& t);

    void identity() {
        rot.x = 1; rot.y = rot.z = rot.w = 0;
        trans.x = trans.y = trans.z = 0;
    }
    
    /// \deprecated (10/07/26) use quatFromAxisAngle
    template <typename U> inline RaveTransform<T>& rotfromaxisangle(const RaveVector<U>& axis, U angle) RAVE_DEPRECATED;

    /// transform a 3 dim vector
    inline RaveVector<T> operator* (const RaveVector<T>& r) const {
        return trans + rotate(r);
    }

    /// transform a vector by the rotation component only
    inline RaveVector<T> rotate(const RaveVector<T>& r) const {
        T xx = 2 * rot.y * rot.y;
        T xy = 2 * rot.y * rot.z;
        T xz = 2 * rot.y * rot.w;
        T xw = 2 * rot.y * rot.x;
        T yy = 2 * rot.z * rot.z;
        T yz = 2 * rot.z * rot.w;
        T yw = 2 * rot.z * rot.x;
        T zz = 2 * rot.w * rot.w;
        T zw = 2 * rot.w * rot.x;

        RaveVector<T> v;
        v.x = (1-yy-zz) * r.x + (xy-zw) * r.y + (xz+yw)*r.z;
        v.y = (xy+zw) * r.x + (1-xx-zz) * r.y + (yz-xw)*r.z;
        v.z = (xz-yw) * r.x + (yz+xw) * r.y + (1-xx-yy)*r.z;
        return v;
    }

    /// transform a transform by the rotation component only
    inline RaveTransform<T> rotate(const RaveTransform<T>& r) const {
        RaveTransform<T> t;
        t.trans = rotate(r.trans);
        t.rot.x = rot.x*r.rot.x - rot.y*r.rot.y - rot.z*r.rot.z - rot.w*r.rot.w;
        t.rot.y = rot.x*r.rot.y + rot.y*r.rot.x + rot.z*r.rot.w - rot.w*r.rot.z;
        t.rot.z = rot.x*r.rot.z + rot.z*r.rot.x + rot.w*r.rot.y - rot.y*r.rot.w;
        t.rot.w = rot.x*r.rot.w + rot.w*r.rot.x + rot.y*r.rot.z - rot.z*r.rot.y;
        // normalize the transformation
        T fnorm = t.rot.lengthsqr4();
        MATH_ASSERT( fnorm > 0.98f && fnorm < 1.02f );
        t.rot /= RaveSqrt(fnorm);
        return t;
    }

    /// t = this * r
    inline RaveTransform<T> operator* (const RaveTransform<T>& r) const {
        RaveTransform<T> t;
        t.trans = operator*(r.trans);
        t.rot.x = rot.x*r.rot.x - rot.y*r.rot.y - rot.z*r.rot.z - rot.w*r.rot.w;
        t.rot.y = rot.x*r.rot.y + rot.y*r.rot.x + rot.z*r.rot.w - rot.w*r.rot.z;
        t.rot.z = rot.x*r.rot.z + rot.z*r.rot.x + rot.w*r.rot.y - rot.y*r.rot.w;
        t.rot.w = rot.x*r.rot.w + rot.w*r.rot.x + rot.y*r.rot.z - rot.z*r.rot.y;
        // normalize the transformation
        T fnorm = t.rot.lengthsqr4();
        MATH_ASSERT( fnorm > 0.98f && fnorm < 1.02f );
        t.rot /= RaveSqrt(fnorm);
        return t;
    }
    
    inline RaveTransform<T>& operator*= (const RaveTransform<T>& right) {
        *this = operator*(right);
        return *this;
    }

    inline RaveTransform<T> inverse() const {
        RaveTransform<T> inv;
        inv.rot.x = rot.x;
        inv.rot.y = -rot.y;
        inv.rot.z = -rot.z;
        inv.rot.w = -rot.w;
        
        inv.trans = -inv.rotate(trans);
        return inv;
    }

    template <typename U> inline RaveTransform<T>& operator= (const RaveTransform<U>& r) {
        trans = r.trans;
        rot = r.rot;
        T fnorm = rot.lengthsqr4();
        MATH_ASSERT( fnorm > 0.99f && fnorm < 1.01f );
        return *this;
    }

    template <typename S, typename U> friend std::basic_ostream<S>& operator<<(std::basic_ostream<S>& O, const RaveTransform<U>& v);
    template <typename S, typename U> friend std::basic_istream<S>& operator>>(std::basic_istream<S>& I, RaveTransform<U>& v);

    RaveVector<T> rot, trans; ///< rot is a quaternion=(cos(ang/2),axisx*sin(ang/2),axisy*sin(ang/2),axisz*sin(ang/2))
};

/** \brief Affine transformation parameterized with rotation matrices. Scales and shears are not supported.
        
    \ingroup affine_math
*/
template <typename T>
class RaveTransformMatrix
{
public:
    inline RaveTransformMatrix() { identity(); m[3] = m[7] = m[11] = 0; }
    template <typename U> RaveTransformMatrix(const RaveTransformMatrix<U>& t) {
        // don't memcpy!
        m[0] = t.m[0]; m[1] = t.m[1]; m[2] = t.m[2]; m[3] = t.m[3];
        m[4] = t.m[4]; m[5] = t.m[5]; m[6] = t.m[6]; m[7] = t.m[7];
        m[8] = t.m[8]; m[9] = t.m[9]; m[10] = t.m[10]; m[11] = t.m[11];
        trans = t.trans;
    }
    inline RaveTransformMatrix(const RaveTransform<T>& t);

    inline void identity() {
        m[0] = 1; m[1] = 0; m[2] = 0;
        m[4] = 0; m[5] = 1; m[6] = 0;
        m[8] = 0; m[9] = 0; m[10] = 1;
        trans.x = trans.y = trans.z = 0;
    }

    /// \deprecated (10/07/26) use matrixFromAxisAngle
    template <typename U> inline RaveTransformMatrix<T>& rotfromaxisangle(const RaveVector<U>& axis, U angle) RAVE_DEPRECATED;

    inline void rotfrommat(T m_00, T m_01, T m_02, T m_10, T m_11, T m_12, T m_20, T m_21, T m_22) {
        m[0] = m_00; m[1] = m_01; m[2] = m_02; m[3] = 0;
        m[4] = m_10; m[5] = m_11; m[6] = m_12; m[7] = 0;
        m[8] = m_20; m[9] = m_21; m[10] = m_22; m[11] = 0;
    }

    inline T rot(int i, int j) const {
        MATH_ASSERT( i >= 0 && i < 3 && j >= 0 && j < 3);
        return m[4*i+j];
    }
    inline T& rot(int i, int j) {
        MATH_ASSERT( i >= 0 && i < 3 && j >= 0 && j < 3);
        return m[4*i+j];
    }
    
    template <typename U>
    inline RaveVector<T> operator* (const RaveVector<U>& r) const {
        RaveVector<T> v;
        v[0] = r[0] * m[0] + r[1] * m[1] + r[2] * m[2] + trans.x;
        v[1] = r[0] * m[4] + r[1] * m[5] + r[2] * m[6] + trans.y;
        v[2] = r[0] * m[8] + r[1] * m[9] + r[2] * m[10] + trans.z;
        return v;
    }

    /// t = this * r
    inline RaveTransformMatrix<T> operator* (const RaveTransformMatrix<T>& r) const {
        RaveTransformMatrix<T> t;
        t.m[0*4+0] = m[0*4+0]*r.m[0*4+0]+m[0*4+1]*r.m[1*4+0]+m[0*4+2]*r.m[2*4+0];
        t.m[0*4+1] = m[0*4+0]*r.m[0*4+1]+m[0*4+1]*r.m[1*4+1]+m[0*4+2]*r.m[2*4+1];
        t.m[0*4+2] = m[0*4+0]*r.m[0*4+2]+m[0*4+1]*r.m[1*4+2]+m[0*4+2]*r.m[2*4+2];
        t.m[1*4+0] = m[1*4+0]*r.m[0*4+0]+m[1*4+1]*r.m[1*4+0]+m[1*4+2]*r.m[2*4+0];
        t.m[1*4+1] = m[1*4+0]*r.m[0*4+1]+m[1*4+1]*r.m[1*4+1]+m[1*4+2]*r.m[2*4+1];
        t.m[1*4+2] = m[1*4+0]*r.m[0*4+2]+m[1*4+1]*r.m[1*4+2]+m[1*4+2]*r.m[2*4+2];
        t.m[2*4+0] = m[2*4+0]*r.m[0*4+0]+m[2*4+1]*r.m[1*4+0]+m[2*4+2]*r.m[2*4+0];
        t.m[2*4+1] = m[2*4+0]*r.m[0*4+1]+m[2*4+1]*r.m[1*4+1]+m[2*4+2]*r.m[2*4+1];
        t.m[2*4+2] = m[2*4+0]*r.m[0*4+2]+m[2*4+1]*r.m[1*4+2]+m[2*4+2]*r.m[2*4+2];
        t.trans[0] = r.trans[0] * m[0] + r.trans[1] * m[1] + r.trans[2] * m[2] + trans.x;
        t.trans[1] = r.trans[0] * m[4] + r.trans[1] * m[5] + r.trans[2] * m[6] + trans.y;
        t.trans[2] = r.trans[0] * m[8] + r.trans[1] * m[9] + r.trans[2] * m[10] + trans.z;
        return t;
    }

    inline RaveTransformMatrix<T> operator*= (const RaveTransformMatrix<T>& r) const {
        *this = *this * r;
        return *this;
    }

    template <typename U>
    inline RaveVector<U> rotate(const RaveVector<U>& r) const {
        RaveVector<U> v;
        v.x = r.x * m[0] + r.y * m[1] + r.z * m[2];
        v.y = r.x * m[4] + r.y * m[5] + r.z * m[6];
        v.z = r.x * m[8] + r.y * m[9] + r.z * m[10];
        return v;
    }
    
    inline RaveTransformMatrix<T> rotate(const RaveTransformMatrix<T>& r) const {
        RaveTransformMatrix<T> t;
        mult3_s4(&t.m[0], &m[0], &r.m[0]);
        t.trans[0] = r.trans[0] * m[0] + r.trans[1] * m[1] + r.trans[2] * m[2];
        t.trans[1] = r.trans[0] * m[4] + r.trans[1] * m[5] + r.trans[2] * m[6];
        t.trans[2] = r.trans[0] * m[8] + r.trans[1] * m[9] + r.trans[2] * m[10];
        return t;
    }

    /// being on the safe side, do the full inverse incase someone uses scaling.
    inline RaveTransformMatrix<T> inverse() const {
        // inverse = C^t / det(pf) where C is the matrix of coefficients
        // calc C^t
        RaveTransformMatrix<T> inv;
        inv.m[0*4+0] = m[1*4 + 1] * m[2*4 + 2] - m[1*4 + 2] * m[2*4 + 1];
        inv.m[0*4+1] = m[0*4 + 2] * m[2*4 + 1] - m[0*4 + 1] * m[2*4 + 2];
        inv.m[0*4+2] = m[0*4 + 1] * m[1*4 + 2] - m[0*4 + 2] * m[1*4 + 1];
        inv.m[1*4+0] = m[1*4 + 2] * m[2*4 + 0] - m[1*4 + 0] * m[2*4 + 2];
        inv.m[1*4+1] = m[0*4 + 0] * m[2*4 + 2] - m[0*4 + 2] * m[2*4 + 0];
        inv.m[1*4+2] = m[0*4 + 2] * m[1*4 + 0] - m[0*4 + 0] * m[1*4 + 2];
        inv.m[2*4+0] = m[1*4 + 0] * m[2*4 + 1] - m[1*4 + 1] * m[2*4 + 0];
        inv.m[2*4+1] = m[0*4 + 1] * m[2*4 + 0] - m[0*4 + 0] * m[2*4 + 1];
        inv.m[2*4+2] = m[0*4 + 0] * m[1*4 + 1] - m[0*4 + 1] * m[1*4 + 0];
        T fdet = m[0*4 + 2] * inv.m[2*4+0] + m[1*4 + 2] * inv.m[2*4+1] + m[2*4 + 2] * inv.m[2*4+2];
        MATH_ASSERT(fdet>=g_fEpsilon);
        fdet = 1 / fdet;
		inv.m[0*4+0] *= fdet;		inv.m[0*4+1] *= fdet;		inv.m[0*4+2] *= fdet;
		inv.m[1*4+0] *= fdet;		inv.m[1*4+1] *= fdet;		inv.m[1*4+2] *= fdet;
		inv.m[2*4+0] *= fdet;		inv.m[2*4+1] *= fdet;		inv.m[2*4+2] *= fdet;
        inv.trans = -inv.rotate(trans);
        return inv;
    }

    template <typename U>
    inline void Extract(RaveVector<U>& right, RaveVector<U>& up, RaveVector<U>& dir, RaveVector<U>& pos) const {
        pos = trans;
        right.x = m[0]; up.x = m[1]; dir.x = m[2];
        right.y = m[4]; up.y = m[5]; dir.y = m[6];
        right.z = m[8]; up.z = m[9]; dir.z = m[10];
    }

    template <typename S, typename U> friend std::basic_ostream<S>& operator<<(std::basic_ostream<S>& O, const RaveTransformMatrix<U>& v);
    template <typename S, typename U> friend std::basic_istream<S>& operator>>(std::basic_istream<S>& I, RaveTransformMatrix<U>& v);

    /// 3x3 rotation matrix. Note that each row is 4 elements long! So row 1 starts at m[4], row 2 at m[8]
    /// The reason is to maintain 16 byte alignment when sizeof(T) is 4 bytes
    T m[12];
    RaveVector<T> trans; ///< translation component
};

/// \brief A ray defined by an origin and a direction.
/// \ingroup geometric_primitives
template <typename T>
class ray
{
public:
    ray() {}
    ray(const RaveVector<T>& _pos, const RaveVector<T>& _dir) : pos(_pos), dir(_dir) {}
    RaveVector<T> pos, dir;
};

/// \brief An axis aligned bounding box.
/// \ingroup geometric_primitives
template <typename T>
class aabb
{
public:
    aabb() {}
    aabb(const RaveVector<T>& vpos, const RaveVector<T>& vextents) : pos(vpos), extents(vextents) {}
    RaveVector<T> pos, extents;
};

/// \brief An oriented bounding box.
/// \ingroup geometric_primitives
template <typename T>
class obb
{
public:
    RaveVector<T> right, up, dir, pos, extents;
};

/// \brief A triangle defined by 3 points.
/// \ingroup geometric_primitives
template <typename T>
class triangle
{
public:
    triangle() {}
    triangle(const RaveVector<T>& v1, const RaveVector<T>& v2, const RaveVector<T>& v3) : v1(v1), v2(v2), v3(v3) {}
    ~triangle() {}

    RaveVector<T> v1, v2, v3;      //!< the vertices of the triangle

    const RaveVector<T>& operator[](int i) const { return (&v1)[i]; }
    RaveVector<T>& operator[](int i)       { return (&v1)[i]; }

    /// assumes CCW ordering of vertices 
    inline RaveVector<T> normal() {
        return (v2-v1).cross(v3-v1);
    }
};

/// \brief A pyramid with its vertex clipped.
/// \ingroup geometric_primitives
template <typename T>
class frustum
{
public:
    RaveVector<T> right, up, dir, pos;
    T fnear, ffar;
    T ffovx,ffovy;
    T fcosfovx,fsinfovx,fcosfovy,fsinfovy;
};

/// \brief Tests a point inside a 3D quadrilateral.
/// \ingroup geometric_primitives
template <typename T>
inline int insideQuadrilateral(const RaveVector<T>& v, const RaveVector<T>& verts)
{
    RaveVector<T> v4,v5;
    T m1,m2;
    T anglesum=0,costheta;
    for (int i=0;i<4;i++) {
        v4.x = verts[i].x - v->x;
        v4.y = verts[i].y - v->y;
        v4.z = verts[i].z - v->z;
        v5.x = verts[(i+1)%4].x - v->x;
        v5.y = verts[(i+1)%4].y - v->y;
        v5.z = verts[(i+1)%4].z - v->z;
        m1 = v4.lengthsqr3();
        m2 = v5.lengthsqr3();
        if (m1*m2 <= g_fEpsilon)
            return(1); // on a vertex, consider this inside
        else {
            costheta = v4.dot3(v5)/RaveSqrt(m1*m2);
        }
        anglesum += RaveAcos(costheta);
    }
    T diff = anglesum - (T)2.0 * PI;
    return diff*diff <= g_fEpsilon*g_fEpsilon;
}

/// \brief Tests a point insdie a 3D triangle.
/// \ingroup geometric_primitives
template <typename T>
inline int insideTriangle(const RaveVector<T> v, const triangle<T>& tri)
{
    RaveVector<T> v4,v5;  
    T m1,m2;
    T anglesum=0.0;
    T costheta;
    for (int i=0;i<3;i++) {
        v4.x = tri[i].x - v->x;
        v4.y = tri[i].y - v->y;
        v4.z = tri[i].z - v->z;
        v5.x = tri[(i+1)%3].x - v->x;
        v5.y = tri[(i+1)%3].y - v->y;
        v5.z = tri[(i+1)%3].z - v->z;
        m1 = v4.lengthsqr3();
        m2 = v5.lengthsqr3();
        if (m1*m2 <= g_fEpsilon) {
    	    return(1); // on a vertex, consider this inside
        }
        else {
    	    costheta = v4.dot3(v5)/RaveSqrt(m1*m2);
        }
        anglesum += acos(costheta);
    }
    T diff = anglesum - (T)2.0 * PI;
    return diff*diff <= g_fEpsilon*g_fEpsilon;
}

/// \brief Test collision of a ray with an axis aligned bounding box.
/// \ingroup geometric_primitives
template <typename T>
inline bool RayAABBTest(const ray<T>& r, const aabb<T>& ab)
{
    RaveVector<T> vd, vpos = r.pos - ab.pos;
    if( RaveFabs(vpos.x) > ab.extents.x && r.dir.x* vpos.x > 0.0f)
        return false;
    if( RaveFabs(vpos.y) > ab.extents.y && r.dir.y * vpos.y > 0.0f)
        return false;
    if( RaveFabs(vpos.z) > ab.extents.z && r.dir.z * vpos.z > 0.0f)
        return false;
    vd = r.dir.cross(vpos);
    if( RaveFabs(vd.x) > ab.extents.y * RaveFabs(r.dir.z) + ab.extents.z * RaveFabs(r.dir.y) )
        return false;
    if( RaveFabs(vd.y) > ab.extents.x * RaveFabs(r.dir.z) + ab.extents.z * RaveFabs(r.dir.x) )
        return false;
    if( RaveFabs(vd.z) > ab.extents.x * RaveFabs(r.dir.y) + ab.extents.y * RaveFabs(r.dir.x) )
        return false;
    return true;
}

/// \brief Test collision of a ray and an oriented bounding box.
/// \ingroup geometric_primitives
template <typename T>
inline bool RayOBBTest(const ray<T>& r, const obb<T>& o)
{
    RaveVector<T> vpos, vdir, vd;
    vd = r.pos - o.pos;
    vpos.x = vd.dot3(o.right);
    vdir.x = r.dir.dot3(o.right);
    if( RaveFabs(vpos.x) > o.extents.x && vdir.x* vpos.x > 0.0f) {
        return false;
    }
    vpos.y = vd.dot3(o.up);
    vdir.y = r.dir.dot3(o.up);
    if( RaveFabs(vpos.y) > o.extents.y && vdir.y * vpos.y > 0.0f) {
        return false;
    }
    vpos.z = vd.dot3(o.dir);
    vdir.z = r.dir.dot3(o.dir);
    if( RaveFabs(vpos.z) > o.extents.z && vdir.z * vpos.z > 0.0f) {
        return false;
    }
    cross3(vd, vdir, vpos);
    if( RaveFabs(vd.x) > o.extents.y * RaveFabs(vdir.z) + o.extents.z * RaveFabs(vdir.y) ||
        RaveFabs(vd.y) > o.extents.x * RaveFabs(vdir.z) + o.extents.z * RaveFabs(vdir.x) ||
        RaveFabs(vd.z) > o.extents.x * RaveFabs(vdir.y) + o.extents.y * RaveFabs(vdir.x) ) {
        return false;
    }
    return true;
}

/// \brief The minimum distance form the vertex to the oriented bounding box.
/// \ingroup geometric_primitives
template <typename T>
T DistVertexOBBSq(const RaveVector<T>& v, const obb<T>& o)
{
    RaveVector<T> vn = v - o.pos;
    vn.x = RaveFabs(vn.dot3(o.right)) - o.extents.x;
    vn.y = RaveFabs(vn.dot3(o.up)) - o.extents.y;
    vn.z = RaveFabs(vn.dot3(o.dir)) - o.extents.z;
    // now we have the vertex in OBB's frame
    T fDist = 0;
    if( vn.x > 0.0f ) {
        fDist += vn.x * vn.x;
    }
    if( vn.y > 0.0f ) {
        fDist += vn.y * vn.y;
    }
    if( vn.z > 0.0f ) {
        fDist += vn.z * vn.z;
    }
    return fDist;
}

/// \brief Test collision of an oriented bounding box and a frustum.
/// \ingroup geometric_primitives
template <typename T>
inline bool IsOBBinFrustum(const obb<T>& o, const frustum<T>& fr)
{
    // check OBB against all 6 planes
    RaveVector<T> v = o.pos - fr.pos;
    // if v lies on the left or bottom sides of the frustrum
    // then freflect about the planes to get it on the right and 
    // top sides
    // side planes
    RaveVector<T> vNorm = fr.fcosfovx * fr.right - fr.fsinfovx * fr.dir;
    if( v.dot3(vNorm) > -o.extents.x * RaveFabs(vNorm.dot3(o.right)) -  o.extents.y * RaveFabs(vNorm.dot3(o.up)) -  o.extents.z * RaveFabs(vNorm.dot3(o.dir))) {
        return false;
    }
    vNorm = -fr.fcosfovx * fr.right - fr.fsinfovx * fr.dir;
    if(dot3(v, vNorm) > -o.extents.x * RaveFabs(vNorm.dot3(o.right)) - o.extents.y * RaveFabs(vNorm.dot3(o.up)) - o.extents.z * RaveFabs(vNorm.dot3(o.dir))) {
        return false;
    }
    vNorm = fr.fcosfovy * fr.up - fr.fsinfovy * fr.dir;
    if(dot3(v, vNorm) > -o.extents.x * RaveFabs(vNorm.dot3(o.right)) - o.extents.y * RaveFabs(vNorm.dot3(o.up)) - o.extents.z * RaveFabs(vNorm.dot3(o.dir))) {
        return false;
    }
    vNorm = -fr.fcosfovy * fr.up - fr.fsinfovy * fr.dir;
    if(dot3(v, vNorm) > -o.extents.x * RaveFabs(vNorm.dot3(o.right)) - o.extents.y * RaveFabs(vNorm.dot3(o.up)) - o.extents.z * RaveFabs(vNorm.dot3(o.dir))) {
        return false;
    }
    vNorm.x = v.dot3(fr.dir);
    vNorm.y = o.extents.x * RaveFabs(fr.dir.dot3(o.right)) +  o.extents.y * RaveFabs(fr.dir.dot3(o.up)) +  o.extents.z * RaveFabs(fr.dir.dot3(o.dir));
    if( (vNorm.x < fr.fnear + vNorm.y) || (vNorm.x > fr.ffar - vNorm.y) ) {
        return false;
    }
    return true;
}

/// \brief Tests if an oriented bounding box is inside a 3D convex hull.
///
/// \ingroup geometric_primitives
/// \param vplanes the plane normals of the convex hull, normals should be facing inside.
template <typename T, typename U>
inline bool IsOBBinConvexHull(const obb<T>& o, const U& vplanes)
{
    for(size_t i = 0; i < vplanes.size(); ++i) {
        RaveVector<T> vplane = vplanes[i];
        if( o.pos.dot3(vplane)+vplane.w < o.extents.x * RaveFabs(vplane.dot3(o.right)) + o.extents.y * RaveFabs(vplane.dot3(o.up)) + o.extents.z * RaveFabs(vplane.dot3(o.dir))) {
            return false;
        }
    }
    return true;
}


/// \brief Test collision if two 3D triangles.
/// \ingroup geometric_primitives
///
/// Assuming triangle vertices are declared counter-clockwise!!
/// \param[out] contactnorm if triangles collide, then filled with the normal of the second triangle
/// \return true if triangles collide.
template <typename T>
inline bool TriTriCollision(const RaveVector<T>& u1, const RaveVector<T>& u2, const RaveVector<T>& u3, const RaveVector<T>& v1, const RaveVector<T>& v2, const RaveVector<T>& v3, RaveVector<T>& contactpos, RaveVector<T>& contactnorm)
{
    // triangle triangle collision test - by Rosen Diankov

    // first see if the faces intersect the planes
    // for the face to be intersecting the plane, one of its
    // vertices must be on the opposite side of the plane
    char b = 0;    
    RaveVector<T> u12 = u2 - u1, u23 = u3 - u2, u31 = u1 - u3;
    RaveVector<T> v12 = v2 - v1, v23 = v3 - v2, v31 = v1 - v3;
    RaveVector<T> vedges[3] = {v12, v23, v31};
    RaveVector<T> unorm, vnorm;
    unorm = u31.cross(u12);
    unorm.w = -unorm.dot3(u1);
    vnorm = v31.cross(v12);
    vnorm.w = -vnorm.dot3(v1);
    if( vnorm.dot3(u1) + vnorm.w > 0 ) {
        b |= 1;
    }
    if( vnorm.dot3(u2) + vnorm.w > 0 ) {
        b |= 2;
    }
    if( vnorm.dot3(u3) + vnorm.w > 0 ) {
        b |= 4;
    }
    if(b == 7 || b == 0) {
        return false;
    }
    // now get segment from f1 when it crosses f2's plane
    // note that b gives us information on which edges actually intersected
    // so figure out the point that is alone on one side of the plane
    // then get the segment
    RaveVector<T> p1, p2;
    const RaveVector<T>* pu=NULL;
    switch(b) {
    case 1:
    case 6:
        pu = &u1;
        p1 = u2 - u1;
        p2 = u3 - u1;
        break;
    case 2:
    case 5:
        pu = &u2;
        p1 = u1 - u2;
        p2 = u3 - u2;
        break;
    case 4:
    case 3:
        pu = &u3;
        p1 = u1 - u3;
        p2 = u2 - u3;
        break;
    }
    
    T t = vnorm.dot3(*pu)+vnorm.w;
    p1 = *pu - p1 * (t / vnorm.dot3(p1));
    p2 = *pu - p2 * (t / vnorm.dot3(p2));

    // go through each of the segments in v2 and clip
    RaveVector<T> vcross;
    const RaveVector<T>* pv[] = {&v1, &v2, &v3, &v1};

    for(int i = 0; i < 3; ++i) {
        const RaveVector<T>* pprev = pv[i];
        RaveVector<T> q1 = p1 - *pprev;
        RaveVector<T> q2 = p2 - *pprev;
        vcross = vedges[i].cross(vnorm);
        T t1 = q1.dot3(vcross);
        T t2 = q2.dot3(vcross);

        // line segment is out of face
        if( t1 >= 0 && t2 >= 0 ) {
            return false;
        }
        if( t1 > 0 && t2 < 0 ) {
            // keep second point, clip first
            RaveVector<T> dq = q2-q1;
            p1 -= dq*(t1/dq.dot3(vcross));
        }
        else if( t1 < 0 && t2 > 0 ) {
            // keep first point, clip second
            RaveVector<T> dq = q1-q2;
            p2 -= dq*(t2/dq.dot3(vcross));
        }
    }

    contactpos = 0.5f * (p1 + p2);
    contactnorm = vnorm.normalize3();
    return true;
}

/// \brief Transform an axis aligned bounding box to an oriented bounding box.
///
/// \ingroup geometric_primitives
/// \param[in] t transformation used to set the coordinate system of ab.
template <typename T>
inline obb<T> OBBFromAABB(const aabb<T>& ab, const RaveTransformMatrix<T>& t)
{
    obb<T> o;
    o.right = RaveVector<T>(t.m[0],t.m[4],t.m[8]);
    o.up = RaveVector<T>(t.m[1],t.m[5],t.m[9]);
    o.dir = RaveVector<T>(t.m[2],t.m[6],t.m[10]);
    o.pos = t*ab.pos;
    o.extents = ab.extents;
    return o;
}

/// \brief Transform an axis aligned bounding box to an oriented bounding box.
///
/// \ingroup geometric_primitives
/// \param[in] t transformation used to set the coordinate system of ab.
template <typename T>
inline obb<T> OBBFromAABB(const aabb<T>& ab, const RaveTransform<T>& t)
{
    return OBBFromAABB(ab,RaveTransformMatrix<T>(t));
}

/// \brief Transforms an oriented bounding box.
///
/// \ingroup geometric_primitives
/// \param[in] t transformation used to set the coordinate system of o.
template <typename T>
inline obb<T> TransformOBB(const RaveTransform<T>& t, const obb<T>& o)
{
    obb<T> newobb;
    newobb.extents = o.extents;
    newobb.pos = t*o.pos;
    newobb.right = t.rotate(o.right);
    newobb.up = t.rotate(o.up);
    newobb.dir = t.rotate(o.dir);
    return newobb;
}

/// \brief Transforms an oriented bounding box.
///
/// \ingroup geometric_primitives
/// \param[in] t transformation used to set the coordinate system of o.
template <typename T>
inline obb<T> TransformOBB(const RaveTransformMatrix<T>& t, const obb<T>& o)
{
    obb<T> newobb;
    newobb.extents = o.extents;
    newobb.pos = t*o.pos;
    newobb.right = t.rotate(o.right);
    newobb.up = t.rotate(o.up);
    newobb.dir = t.rotate(o.dir);
    return newobb;
}

/// \brief Test collision between two axis-aligned bounding boxes.
///
/// \ingroup geometric_primitives
template <typename T>
inline bool AABBCollision(const aabb<T>& ab1, const aabb<T>& ab2)
{
    RaveVector<T> v = ab1.pos-ab2.pos;
    return RaveFabs(v.x) <= ab1.extents.x+ab2.extents.x && RaveFabs(v.y) <= ab1.extents.y+ab2.extents.y && RaveFabs(v.z) <= ab1.extents.z+ab2.extents.z;
}

/// \brief Converts an axis-angle rotation into a quaternion.
///
/// \ingroup affine_math
/// \param axis unit axis, 3 values
/// \param angle rotation angle (radians)
template <typename T> inline RaveVector<T> quatFromAxisAngle(const RaveVector<T>& axis, T angle)
{
    T axislen = RaveSqrt(axis.lengthsqr3());
    if( axislen == 0 ) {
        return RaveVector<T>(T(1),T(0),T(0),T(0));
    }
    angle *= T(0.5);
    T sang = RaveSin(angle)/axislen;
    return RaveVector<T>(RaveCos(angle),axis.x*sang,axis.y*sang,axis.z*sang);
}

/// \brief Converts an axis-angle rotation into a quaternion.
///
/// \ingroup affine_math
/// \param axisangle unit axis * rotation angle (radians), 3 values
template <typename T> inline RaveVector<T> quatFromAxisAngle(const RaveVector<T>& axisangle)
{
    T axislen = RaveSqrt(axisangle.lengthsqr3());
    if( axislen < g_fEpsilon ) {
        return RaveVector<T>(T(1),T(0),T(0),T(0));
    }
    T sang = RaveSin(axislen*T(0.5))/axislen;
    return RaveVector<T>(RaveCos(axislen*T(0.5)),axisangle.x*sang,axisangle.y*sang,axisangle.z*sang);
}

/// \brief Converts the rotation of a matrix into a quaternion.
///
/// \ingroup affine_math
/// \param t transform for extracting the 3x3 rotation.
template <typename T> inline RaveVector<T> quatFromMatrix(const RaveTransformMatrix<T>& rotation)
{
    RaveVector<T> rot;
    T tr = rotation.m[4*0+0] + rotation.m[4*1+1] + rotation.m[4*2+2];
    if (tr >= 0) {
        rot[0] = tr + 1;
        rot[1] = (rotation.m[4*2+1] - rotation.m[4*1+2]);
        rot[2] = (rotation.m[4*0+2] - rotation.m[4*2+0]);
        rot[3] = (rotation.m[4*1+0] - rotation.m[4*0+1]);
    }
    else {
        // find the largest diagonal element and jump to the appropriate case
        if (rotation.m[4*1+1] > rotation.m[4*0+0]) {
            if (rotation.m[4*2+2] > rotation.m[4*1+1]) {
                rot[3] = (rotation.m[4*2+2] - (rotation.m[4*0+0] + rotation.m[4*1+1])) + 1;
                rot[1] = (rotation.m[4*2+0] + rotation.m[4*0+2]);
                rot[2] = (rotation.m[4*1+2] + rotation.m[4*2+1]);
                rot[0] = (rotation.m[4*1+0] - rotation.m[4*0+1]);
            }
            else {
                rot[2] = (rotation.m[4*1+1] - (rotation.m[4*2+2] + rotation.m[4*0+0])) + 1;
                rot[3] = (rotation.m[4*1+2] + rotation.m[4*2+1]);
                rot[1] = (rotation.m[4*0+1] + rotation.m[4*1+0]);
                rot[0] = (rotation.m[4*0+2] - rotation.m[4*2+0]);
            }
        }
        else if (rotation.m[4*2+2] > rotation.m[4*0+0]) {
            rot[3] = (rotation.m[4*2+2] - (rotation.m[4*0+0] + rotation.m[4*1+1])) + 1;
            rot[1] = (rotation.m[4*2+0] + rotation.m[4*0+2]);
            rot[2] = (rotation.m[4*1+2] + rotation.m[4*2+1]);
            rot[0] = (rotation.m[4*1+0] - rotation.m[4*0+1]);
        }
        else {
            rot[1] = (rotation.m[4*0+0] - (rotation.m[4*1+1] + rotation.m[4*2+2])) + 1;
            rot[2] = (rotation.m[4*0+1] + rotation.m[4*1+0]);
            rot[3] = (rotation.m[4*2+0] + rotation.m[4*0+2]);
            rot[0] = (rotation.m[4*2+1] - rotation.m[4*1+2]);
        }
    }
    return rot * (T(1) / RaveSqrt(rot.lengthsqr4()));
}

/// \brief Converts a quaternion to a 3x3 matrix
///
/// \ingroup affine_math
/// \param[in] quat quaternion, (s,vx,vy,vz)
template <typename T> inline RaveTransformMatrix<T> matrixFromQuat(const RaveVector<T>& quat)
{
    RaveTransformMatrix<T> t;
    T qq1 = 2*quat[1]*quat[1];
    T qq2 = 2*quat[2]*quat[2];
    T qq3 = 2*quat[3]*quat[3];
    t.m[4*0+0] = 1 - qq2 - qq3;
    t.m[4*0+1] = 2*(quat[1]*quat[2] - quat[0]*quat[3]);
    t.m[4*0+2] = 2*(quat[1]*quat[3] + quat[0]*quat[2]);
    t.m[4*0+3] = 0;
    t.m[4*1+0] = 2*(quat[1]*quat[2] + quat[0]*quat[3]);
    t.m[4*1+1] = 1 - qq1 - qq3;
    t.m[4*1+2] = 2*(quat[2]*quat[3] - quat[0]*quat[1]);
    t.m[4*1+3] = 0;
    t.m[4*2+0] = 2*(quat[1]*quat[3] - quat[0]*quat[2]);
    t.m[4*2+1] = 2*(quat[2]*quat[3] + quat[0]*quat[1]);
    t.m[4*2+2] = 1 - qq1 - qq2;
    t.m[4*2+3] = 0;
    return t;
}

/// \brief Converts a quaternion to a 3x3 matrix
///
/// \ingroup affine_math
/// \param[out] rotation 
/// \param[in] quat quaternion, (s,vx,vy,vz)
template <typename T> void matrixFromQuat(RaveTransformMatrix<T>& rotation, const RaveVector<T>& quat)
{
    T qq1 = 2*quat[1]*quat[1];
    T qq2 = 2*quat[2]*quat[2];
    T qq3 = 2*quat[3]*quat[3];
    rotation.m[4*0+0] = 1 - qq2 - qq3;
    rotation.m[4*0+1] = 2*(quat[1]*quat[2] - quat[0]*quat[3]);
    rotation.m[4*0+2] = 2*(quat[1]*quat[3] + quat[0]*quat[2]);
    rotation.m[4*0+3] = 0;
    rotation.m[4*1+0] = 2*(quat[1]*quat[2] + quat[0]*quat[3]);
    rotation.m[4*1+1] = 1 - qq1 - qq3;
    rotation.m[4*1+2] = 2*(quat[2]*quat[3] - quat[0]*quat[1]);
    rotation.m[4*1+3] = 0;
    rotation.m[4*2+0] = 2*(quat[1]*quat[3] - quat[0]*quat[2]);
    rotation.m[4*2+1] = 2*(quat[2]*quat[3] + quat[0]*quat[1]);
    rotation.m[4*2+2] = 1 - qq1 - qq2;
    rotation.m[4*2+3] = 0;
}

/// \brief Converts an axis-angle rotation to a 3x3 matrix.
///
/// \ingroup affine_math
/// \param axis unit axis, 3 values
/// \param angle rotation angle (radians)
template <typename T> inline RaveTransformMatrix<T> matrixFromAxisAngle(const RaveVector<T>& axis, T angle)
{
    return matrixFromQuat(quatFromAxisAngle(axis,angle));
}

/// \brief Converts an axis-angle rotation to a 3x3 matrix.
///
/// \ingroup affine_math
/// \param axis unit axis * rotation angle (radians), 3 values
template <typename T> inline RaveTransformMatrix<T> matrixFromAxisAngle(const RaveVector<T>& axisangle)
{
    return matrixFromQuat(quatFromAxisAngle(axisangle));
}

/// \brief Multiply two quaternions
///
/// \ingroup affine_math
/// \param quat0 quaternion, (s,vx,vy,vz)
/// \param quat1 quaternion, (s,vx,vy,vz)
template <typename T>
inline RaveVector<T> quatMultiply(const RaveVector<T>& quat0, const RaveVector<T>& quat1)
{
    RaveVector<T> q(quat0.x*quat1.x - quat0.y*quat1.y - quat0.z*quat1.z - quat0.w*quat1.w,
                    quat0.x*quat1.y + quat0.y*quat1.x + quat0.z*quat1.w - quat0.w*quat1.z,
                    quat0.x*quat1.z + quat0.z*quat1.x + quat0.w*quat1.y - quat0.y*quat1.w,
                    quat0.x*quat1.w + quat0.w*quat1.x + quat0.y*quat1.z - quat0.z*quat1.y);
    T fnorm = q.lengthsqr4(); // normalize the quaternion
    MATH_ASSERT( fnorm > 0.98f && fnorm < 1.02f ); // catches multi-threading errors
    return q * (T(1)/RaveSqrt(fnorm));
}

/// \brief Inverted a quaternion rotation.
///
/// \ingroup affine_math
/// \param quat quaternion, (s,vx,vy,vz)
template <typename T>
inline RaveVector<T> quatInverse(const RaveVector<T>& quat)
{
    return RaveVector<T>(quat.x,-quat.y,-quat.z,-quat.w);
}

/// \brief Sphereical linear interpolation between two quaternions.
///
/// \ingroup affine_math
/// \param quat0 quaternion, (s,vx,vy,vz)
/// \param quat1 quaternion, (s,vx,vy,vz)
/// \param t real value in [0,1]. 0 returns quat1, 1 returns quat2
template <typename T>
inline RaveVector<T> quatSlerp(const RaveVector<T>& quat0, const RaveVector<T>& quat1, T t)
{
    // quaternion to return
    RaveVector<T> qb, qm;
    if( quat0.dot(quat1) < 0 ) {
        qb = -quat1;
    }
    else {
        qb = quat1;
    }
    // Calculate angle between them.
    T cosHalfTheta = quat0.w * qb.w + quat0.x * qb.x + quat0.y * qb.y + quat0.z * qb.z;
    // if quat0=qb or quat0=-qb then theta = 0 and we can return quat0
    if (RaveFabs(cosHalfTheta) >= 1.0){
        qm.w = quat0.w;qm.x = quat0.x;qm.y = quat0.y;qm.z = quat0.z;
        return qm;
    }
    // Calculate temporary values.
    T halfTheta = RaveAcos(cosHalfTheta);
    T sinHalfTheta = RaveSqrt(1 - cosHalfTheta*cosHalfTheta);
    // if theta = 180 degrees then result is not fully defined
    // we could rotate around any axis normal to quat0 or qb
    if (RaveFabs(sinHalfTheta) < 1e-7f){ // fabs is floating point absolute
        qm.w = (quat0.w * 0.5f + qb.w * 0.5f);
        qm.x = (quat0.x * 0.5f + qb.x * 0.5f);
        qm.y = (quat0.y * 0.5f + qb.y * 0.5f);
        qm.z = (quat0.z * 0.5f + qb.z * 0.5f);
        return qm;
    }
    T ratioA = RaveSin((1 - t) * halfTheta) / sinHalfTheta;
    T ratioB = RaveSin(t * halfTheta) / sinHalfTheta; 
    //calculate Quaternion.
    qm.w = (quat0.w * ratioA + qb.w * ratioB);
    qm.x = (quat0.x * ratioA + qb.x * ratioB);
    qm.y = (quat0.y * ratioA + qb.y * ratioB);
    qm.z = (quat0.z * ratioA + qb.z * ratioB);
    return qm;
}

/// \deprecated (10/07/26) use quatSlerp
template <typename T>
inline RaveVector<T> dQSlerp(const RaveVector<T>& qa, const RaveVector<T>& _qb, T t) RAVE_DEPRECATED;

template <typename T>
inline RaveVector<T> dQSlerp(const RaveVector<T>& qa, const RaveVector<T>& _qb, T t)
{
    return quatSlerp<T>(qa,_qb,t);
}

/// \deprecated (10/07/26) use quatFromAxisAngle
template <typename T> inline RaveVector<T> AxisAngle2Quat(const RaveVector<T>& rotaxis, T angle) RAVE_DEPRECATED;

template <typename T> inline RaveVector<T> AxisAngle2Quat(const RaveVector<T>& rotaxis, T angle)
{
    angle *= (T)0.5;
    T fsin = RaveSin(angle);
    return RaveVector<T>(RaveCos(angle), rotaxis.x*fsin, rotaxis.y * fsin, rotaxis.z * fsin);
}

/// \brief Return the minimal quaternion that orients sourcedir to targetdir
///
/// \ingroup affine_math
/// \param sourcedir direction of the original vector, 3 values
/// \param targetdir new direction, 3 values
template<typename T>
RaveVector<T> quatRotateDirection(const RaveVector<T>& sourcedir, const RaveVector<T>& targetdir)
{
    RaveVector<T> rottodirection = sourcedir.cross(targetdir);
    T fsin = RaveSqrt(rottodirection.lengthsqr3());
    T fcos = sourcedir.dot3(targetdir);
    RaveTransform<T> torient;
    if( fsin > g_fEpsilon ) {
        return quatFromAxisAngle(rottodirection*(1/fsin), RaveAtan2(fsin, fcos));
    }
    if( fcos < 0 ) {
        // hand is flipped 180, rotate around x axis
        rottodirection = RaveVector<T>(1,0,0);
        rottodirection -= sourcedir * sourcedir.dot3(rottodirection);
        if( rottodirection.lengthsqr3()<1e-8 ) {
            rottodirection = RaveVector<T>(0,0,1);
            rottodirection -= sourcedir * sourcedir.dot3(rottodirection);
        }
        rottodirection.normalize3();
        return quatFromAxisAngle(rottodirection, RaveAtan2(fsin, fcos));
    }
    return RaveVector<T>(T(1),T(0),T(0),T(0));
}

/// \brief Converts a quaternion into the axis-angle representation.
///
/// \ingroup affine_math
/// \param quat quaternion, (s,vx,vy,vz)
template<typename T>
RaveVector<T> axisAngleFromQuat(const RaveVector<T>& quat)
{
    T sinang = quat.y*quat.y+quat.z*quat.z+quat.w*quat.w;
    if( RaveFabs(sinang) < g_fEpsilon ) {
        return RaveVector<T>(0,0,0);
    }
    RaveVector<T> _quat;
    if( quat.x < 0 ) {
        _quat = -quat;
    }
    else {
        _quat = quat;
    }
    sinang = RaveSqrt(sinang);
    T f = 2.0*RaveAtan2(sinang,_quat.x)/sinang;
    return RaveVector<T>(_quat.y*f,_quat.z*f,_quat.w*f);
}

/// \brief Converts the rotation of a matrix into axis-angle representation.
///
/// \ingroup affine_math
/// \param rotation 3x3 rotation matrix
template<typename T>
RaveVector<T> axisAngleFromMatrix(const RaveTransformMatrix<T>& rotation)
{
    return axisAngleFromQuat(quatFromMatrix(rotation));
}

template <typename T>
RaveTransform<T>::RaveTransform(const RaveTransformMatrix<T>& t)
{
    trans = t.trans;
    rot = quatFromMatrix(t);
}

template <typename T>
RaveTransformMatrix<T>::RaveTransformMatrix(const RaveTransform<T>& t)
{
    matrixFromQuat(*this, t.rot);
    trans = t.trans;

}

template <typename T> template <typename U> inline RaveTransformMatrix<T>& RaveTransformMatrix<T>::rotfromaxisangle(const RaveVector<U>& axis, U angle)
{
    *this = matrixFromAxisAngle(axis,angle);
}

template <typename T> template <typename U> inline RaveTransform<T>& RaveTransform<T>::rotfromaxisangle(const RaveVector<U>& axis, U angle) {
    U sinang = (U)RaveSin(angle/2);
    rot.x = (U)RaveCos(angle/2);
    rot.y = axis.x*sinang;
    rot.z = axis.y*sinang;
    rot.w = axis.z*sinang;
    T fnorm = rot.lengthsqr4();
    MATH_ASSERT( fnorm > 0.99f && fnorm < 1.01f );
    return *this;
}
/// \brief Returns a camera matrix that looks along a ray with a desired up vector.
///
/// \ingroup affine_math
/// \param[in] vlookat the point space to look at, the camera will rotation and zoom around this point
/// \param[in] vcampos the position of the camera in space
/// \param[in] vcamup vector from the camera
template<typename T>
RaveTransformMatrix<T> transformLookat(const RaveVector<T>& vlookat, const RaveVector<T>& vcamerapos, const RaveVector<T>& vcameraup)
{
    RaveVector<T> dir = vlookat - vcamerapos;
    T len = RaveSqrt(dir.lengthsqr3());
    if( len > 1e-6 ) {
        dir *= 1/len;
    }
    else {
        dir = RaveVector<T>(0,0,1);
    }
    RaveVector<T> up = vcameraup - dir * dir.dot3(vcameraup);
    len = up.lengthsqr3();
    if( len < 1e-8 ) {
        up = RaveVector<T>(0,1,0);
        up -= dir * dir.dot3(up);
        len = up.lengthsqr3();
        if( len < 1e-8 ) {
            up = RaveVector<T>(1,0,0);
            up -= dir * dir.dot3(up);
            len = up.lengthsqr3();
        }
    }
    up *= 1/RaveSqrt(len);
    RaveVector<T> right = up.cross(dir);
    RaveTransformMatrix<T> t;
    t.m[0] = right.x; t.m[1] = up.x; t.m[2] = dir.x;
    t.m[4] = right.y; t.m[5] = up.y; t.m[6] = dir.y;
    t.m[8] = right.z; t.m[9] = up.z; t.m[10] = dir.z;
    t.trans = vcamerapos;
    return t;
}

// Don't add new lines to the output << operators. Some applications use it to serialize the data
// to send across the network.

template <typename T, typename U>
std::basic_ostream<T>& operator<<(std::basic_ostream<T>& O, const RaveVector<U>& v)
{
    return O << v.x << " " << v.y << " " << v.z << " " << v.w << " ";
}

template <typename T, typename U>
std::basic_istream<T>& operator>>(std::basic_istream<T>& I, RaveVector<U>& v)
{
    return I >> v.x >> v.y >> v.z >> v.w;
}

template <typename T, typename U>
std::basic_ostream<T>& operator<<(std::basic_ostream<T>& O, const RaveTransform<U>& v)
{
    return O << v.rot.x << " " << v.rot.y << " " << v.rot.z << " " << v.rot.w << " "
             << v.trans.x << " " << v.trans.y << " " << v.trans.z << " ";
}

template <typename T, typename U>
std::basic_istream<T>& operator>>(std::basic_istream<T>& I, RaveTransform<U>& v)
{
    return I >> v.rot.x >> v.rot.y >> v.rot.z >> v.rot.w >> v.trans.x >> v.trans.y >> v.trans.z;
}

// serial in column order! This is the format transformations are passed across the network
template <typename T, typename U>
std::basic_ostream<T>& operator<<(std::basic_ostream<T>& O, const RaveTransformMatrix<U>& v)
{
    return O << v.m[0] << " " << v.m[4] << " " << v.m[8] << " "
             << v.m[1] << " " << v.m[5] << " " << v.m[9] << " "
             << v.m[2] << " " << v.m[6] << " " << v.m[10] << " "
             << v.trans.x << " " << v.trans.y << " " << v.trans.z << " ";
}

// read in column order! This is the format transformations are passed across the network
template <typename T, typename U>
std::basic_istream<T>& operator>>(std::basic_istream<T>& I, RaveTransformMatrix<U>& v)
{
    return I >> v.m[0] >> v.m[4] >> v.m[8]
             >> v.m[1] >> v.m[5] >> v.m[9]
             >> v.m[2] >> v.m[6] >> v.m[10]
             >> v.trans.x >> v.trans.y >> v.trans.z;
}

} // end namespace geometry
} // end namespace OpenRAVE

#endif
