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
    \brief  Basic gemoetric primitives and functions on them.
 */

#ifndef OPENRAVE_GEOMETRY_H
#define OPENRAVE_GEOMETRY_H

#include <cmath>
#include <iostream>
#include <cstring>
#include <climits>
#include <cstdlib>

#ifndef RAVE_API
#define RAVE_API
#endif

#ifdef BOOST_ASSERT
#define MATH_ASSERT BOOST_ASSERT
#else
#include <cassert>
#define MATH_ASSERT assert
#endif

#ifndef MATH_RANDOM_FLOAT
#define MATH_RANDOM_FLOAT (rand()/((T)RAND_MAX))
#endif

namespace OpenRAVE {

/// Templated math and geometric functions
namespace geometry {

#ifndef PI
#define PI 3.14159265358979
#endif

#define distinctRoots	0			// roots r0 < r1 < r2
#define singleRoot		1			// root r0
#define floatRoot01		2			// roots r0 = r1 < r2
#define floatRoot12		4			// roots r0 < r1 = r2
#define tripleRoot		6			// roots r0 = r1 = r2

template <class T> class RaveTransform;
template <class T> class RaveTransformMatrix;

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
template <class T>
class RaveVector
{
public:
    T x, y, z, w;
    
    RaveVector() : x(0), y(0), z(0), w(0) {}
        
    RaveVector(T x, T y, T z) : x(x), y(y), z(z), w(0) {}
    RaveVector(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {}
    template<class U> RaveVector(const RaveVector<U> &vec) : x((T)vec.x), y((T)vec.y), z((T)vec.z), w((T)vec.w) {}

    /// note, it only copes 3 values!
    template<class U> RaveVector(const U* pf) { MATH_ASSERT(pf != NULL); x = (T)pf[0]; y = (T)pf[1]; z = (T)pf[2]; w = 0; }
    
    T  operator[](int i) const       { return (&x)[i]; }
    T& operator[](int i)             { return (&x)[i]; }
    
    // casting operators
    operator T* () { return &x; }
    operator const T* () const { return (const T*)&x; }

    template <class U>
    RaveVector<T>& operator=(const RaveVector<U>& r) { x = (T)r.x; y = (T)r.y; z = (T)r.z; w = (T)r.w; return *this; }
    
    // SCALAR FUNCTIONS
    template <class U> inline T dot(const RaveVector<U> &v) const { return x*v.x + y*v.y + z*v.z + w*v.w; }
    template <class U> inline T dot3(const RaveVector<U> &v) const { return x*v.x + y*v.y + z*v.z; }
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
    template <class U> inline RaveVector<T> operator+(const RaveVector<U> &r) const { RaveVector<T> v; v.x = x+r.x; v.y = y+r.y; v.z = z+r.z; v.w = w+r.w; return v; }
    template <class U> inline RaveVector<T> operator-(const RaveVector<U> &r) const { RaveVector<T> v; v.x = x-r.x; v.y = y-r.y; v.z = z-r.z; v.w = w-r.w; return v; }
    template <class U> inline RaveVector<T> operator*(const RaveVector<U> &r) const { RaveVector<T> v; v.x = r.x*x; v.y = r.y*y; v.z = r.z*z; v.w = r.w*w; return v; }
    inline RaveVector<T> operator*(T k) const { RaveVector<T> v; v.x = k*x; v.y = k*y; v.z = k*z; v.w = k*w; return v; }

    template <class U> inline RaveVector<T>& operator += (const RaveVector<U>& r) { x += r.x; y += r.y; z += r.z; w += r.w; return *this; }
    template <class U> inline RaveVector<T>& operator -= (const RaveVector<U>& r) { x -= r.x; y -= r.y; z -= r.z; w -= r.w; return *this; }
    template <class U> inline RaveVector<T>& operator *= (const RaveVector<U>& r) { x *= r.x; y *= r.y; z *= r.z; w *= r.w; return *this; }
    
    inline RaveVector<T>& operator *= (const T k) { x *= k; y *= k; z *= k; w *= k; return *this; }
    inline RaveVector<T>& operator /= (const T _k) { T k=1/_k; x *= k; y *= k; z *= k; w *= k; return *this; }

    template <class U> friend RaveVector<U> operator* (float f, const RaveVector<U>& v);
    template <class U> friend RaveVector<U> operator* (double f, const RaveVector<U>& v);
    
    template <class S, class U> friend std::basic_ostream<S>& operator<<(std::basic_ostream<S>& O, const RaveVector<U>& v);
    template <class S, class U> friend std::basic_istream<S>& operator>>(std::basic_istream<S>& I, RaveVector<U>& v);

    /// cross product operator
    template <class U> inline RaveVector<T> operator^(const RaveVector<U> &v) const { 
        RaveVector<T> ucrossv;
        ucrossv[0] = y * v[2] - z * v[1];
        ucrossv[1] = z * v[0] - x * v[2];
        ucrossv[2] = x * v[1] - y * v[0];
        return ucrossv;
    }
};

template <class T>
inline RaveVector<T> operator* (float f, const RaveVector<T>& left)
{
    RaveVector<T> v;
    v.x = (T)f * left.x;
    v.y = (T)f * left.y;
    v.z = (T)f * left.z;
    v.w = (T)f * left.w;
    return v;
}

template <class T>
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
template <class T>
class RaveTransform
{
public:
    RaveTransform() { rot.x = 1; }
    template <class U> RaveTransform(const RaveTransform<U>& t) {
        rot = t.rot;
        trans = t.trans;
        T fnorm = rot.lengthsqr4();
        MATH_ASSERT( fnorm > 0.99f && fnorm < 1.01f );
    }
    template <class U> RaveTransform(const RaveVector<U>& rot, const RaveVector<U>& trans) : rot(rot), trans(trans) {
        T fnorm = rot.lengthsqr4();
        MATH_ASSERT( fnorm > 0.99f && fnorm < 1.01f );
    }
    inline RaveTransform(const RaveTransformMatrix<T>& t);

    void identity() {
        rot.x = 1; rot.y = rot.z = rot.w = 0;
        trans.x = trans.y = trans.z = 0;
    }
    
    template <class U> inline RaveTransform<T>& rotfromaxisangle(const RaveVector<U>& axis, U angle) {
        U sinang = (U)RaveSin(angle/2);
        rot.x = (U)RaveCos(angle/2);
        rot.y = axis.x*sinang;
        rot.z = axis.y*sinang;
        rot.w = axis.z*sinang;
        T fnorm = rot.lengthsqr4();
        MATH_ASSERT( fnorm > 0.99f && fnorm < 1.01f );
        return *this;
    }

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

    template <class U> inline RaveTransform<T>& operator= (const RaveTransform<U>& r) {
        trans = r.trans;
        rot = r.rot;
        T fnorm = rot.lengthsqr4();
        MATH_ASSERT( fnorm > 0.99f && fnorm < 1.01f );
        return *this;
    }

    template <class S, class U> friend std::basic_ostream<S>& operator<<(std::basic_ostream<S>& O, const RaveTransform<U>& v);
    template <class S, class U> friend std::basic_istream<S>& operator>>(std::basic_istream<S>& I, RaveTransform<U>& v);

    RaveVector<T> rot, trans; ///< rot is a quaternion=(cos(ang/2),axisx*sin(ang/2),axisy*sin(ang/2),axisz*sin(ang/2))
};

/** \brief Affine transformation parameterized with rotation matrices.
        
    \ingroup affine_math
*/
template <class T>
class RaveTransformMatrix
{
public:
    inline RaveTransformMatrix() { identity(); m[3] = m[7] = m[11] = 0; }
    template <class U> RaveTransformMatrix(const RaveTransformMatrix<U>& t) {
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
    
    template <class U> inline RaveTransformMatrix<T>& rotfromaxisangle(const RaveVector<U>& axis, U angle) {
        RaveVector<T> quat;
        U sinang = (U)RaveSin(angle/2);
        quat.x = (U)RaveCos(angle/2);
        quat.y = axis.x*sinang;
        quat.z = axis.y*sinang;
        quat.w = axis.z*sinang;
        rotfromquat(quat);
        return *this;
    }

    template <class U>
    inline void rotfromquat(const RaveVector<U>& quat) {
            // q = (s,vx,vy,vz)
        T qq1 = 2*quat[1]*quat[1];
        T qq2 = 2*quat[2]*quat[2];
        T qq3 = 2*quat[3]*quat[3];
        m[4*0+0] = 1 - qq2 - qq3;
        m[4*0+1] = 2*(quat[1]*quat[2] - quat[0]*quat[3]);
        m[4*0+2] = 2*(quat[1]*quat[3] + quat[0]*quat[2]);
        m[4*0+3] = 0;
        m[4*1+0] = 2*(quat[1]*quat[2] + quat[0]*quat[3]);
        m[4*1+1] = 1 - qq1 - qq3;
        m[4*1+2] = 2*(quat[2]*quat[3] - quat[0]*quat[1]);
        m[4*1+3] = 0;
        m[4*2+0] = 2*(quat[1]*quat[3] - quat[0]*quat[2]);
        m[4*2+1] = 2*(quat[2]*quat[3] + quat[0]*quat[1]);
        m[4*2+2] = 1 - qq1 - qq2;
        m[4*2+3] = 0;
    }
    
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
    
    template <class U>
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
        mult3_s4(&t.m[0], &m[0], &r.m[0]);
        t.trans[0] = r.trans[0] * m[0] + r.trans[1] * m[1] + r.trans[2] * m[2] + trans.x;
        t.trans[1] = r.trans[0] * m[4] + r.trans[1] * m[5] + r.trans[2] * m[6] + trans.y;
        t.trans[2] = r.trans[0] * m[8] + r.trans[1] * m[9] + r.trans[2] * m[10] + trans.z;
        return t;
    }

    inline RaveTransformMatrix<T> operator*= (const RaveTransformMatrix<T>& r) const {
        *this = *this * r;
        return *this;
    }

    template <class U>
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

    inline RaveTransformMatrix<T> inverse() const {
        RaveTransformMatrix<T> inv;
        inv3(m, inv.m, NULL, 4);
        inv.trans = -inv.rotate(trans);
        return inv;
    }

    template <class U>
    inline void Extract(RaveVector<U>& right, RaveVector<U>& up, RaveVector<U>& dir, RaveVector<U>& pos) const {
        pos = trans;
        right.x = m[0]; up.x = m[1]; dir.x = m[2];
        right.y = m[4]; up.y = m[5]; dir.y = m[6];
        right.z = m[8]; up.z = m[9]; dir.z = m[10];
    }

    template <class S, class U> friend std::basic_ostream<S>& operator<<(std::basic_ostream<S>& O, const RaveTransformMatrix<U>& v);
    template <class S, class U> friend std::basic_istream<S>& operator>>(std::basic_istream<S>& I, RaveTransformMatrix<U>& v);

    /// 3x3 rotation matrix. Note that each row is 4 elements long! So row 1 starts at m[4], row 2 at m[8]
    /// The reason is to maintain 16 byte alignment when sizeof(T) is 4 bytes
    T m[12];
    RaveVector<T> trans; ///< translation component
};

template <class T>
RaveTransform<T>::RaveTransform(const RaveTransformMatrix<T>& t)
{
    trans = t.trans;
    T tr = t.m[4*0+0] + t.m[4*1+1] + t.m[4*2+2];
    if (tr >= 0) {
        rot[0] = tr + 1;
        rot[1] = (t.m[4*2+1] - t.m[4*1+2]);
        rot[2] = (t.m[4*0+2] - t.m[4*2+0]);
        rot[3] = (t.m[4*1+0] - t.m[4*0+1]);
    }
    else {
        // find the largest diagonal element and jump to the appropriate case
        if (t.m[4*1+1] > t.m[4*0+0]) {
            if (t.m[4*2+2] > t.m[4*1+1]) {
                rot[3] = (t.m[4*2+2] - (t.m[4*0+0] + t.m[4*1+1])) + 1;
                rot[1] = (t.m[4*2+0] + t.m[4*0+2]);
                rot[2] = (t.m[4*1+2] + t.m[4*2+1]);
                rot[0] = (t.m[4*1+0] - t.m[4*0+1]);
            }
            else {
                rot[2] = (t.m[4*1+1] - (t.m[4*2+2] + t.m[4*0+0])) + 1;
                rot[3] = (t.m[4*1+2] + t.m[4*2+1]);
                rot[1] = (t.m[4*0+1] + t.m[4*1+0]);
                rot[0] = (t.m[4*0+2] - t.m[4*2+0]);
            }
        }
        else if (t.m[4*2+2] > t.m[4*0+0]) {
            rot[3] = (t.m[4*2+2] - (t.m[4*0+0] + t.m[4*1+1])) + 1;
            rot[1] = (t.m[4*2+0] + t.m[4*0+2]);
            rot[2] = (t.m[4*1+2] + t.m[4*2+1]);
            rot[0] = (t.m[4*1+0] - t.m[4*0+1]);
        }
        else {
            rot[1] = (t.m[4*0+0] - (t.m[4*1+1] + t.m[4*2+2])) + 1;
            rot[2] = (t.m[4*0+1] + t.m[4*1+0]);
            rot[3] = (t.m[4*2+0] + t.m[4*0+2]);
            rot[0] = (t.m[4*2+1] - t.m[4*1+2]);
        }
    }

    rot *= (T)1 / RaveSqrt(rot.lengthsqr4());
}

template <class T>
RaveTransformMatrix<T>::RaveTransformMatrix(const RaveTransform<T>& t)
{
    rotfromquat(t.rot);
    trans = t.trans;

}

/// \brief A ray defined by an origin and a direction.
/// \ingroup geometric_primitives
template <typename T>
struct ray
{
    ray() {}
    ray(const RaveVector<T>& _pos, const RaveVector<T>& _dir) : pos(_pos), dir(_dir) {}
    RaveVector<T> pos, dir;
};

/// \brief An axis aligned bounding box.
/// \ingroup geometric_primitives
template <typename T>
struct aabb
{
    aabb() {}
    aabb(const RaveVector<T>& vpos, const RaveVector<T>& vextents) : pos(vpos), extents(vextents) {}
    RaveVector<T> pos, extents;
};

/// \brief An oriented bounding box.
/// \ingroup geometric_primitives
template <typename T>
struct obb
{
    RaveVector<T> right, up, dir, pos, extents;
};

/// \brief A triangle defined by 3 points.
/// \ingroup geometric_primitives
template <typename T>
struct triangle
{
    triangle() {}
    triangle(const RaveVector<T>& v1, const RaveVector<T>& v2, const RaveVector<T>& v3) : v1(v1), v2(v2), v3(v3) {}
    ~triangle() {}

    RaveVector<T> v1, v2, v3;      //!< the vertices of the triangle

    const RaveVector<T>& operator[](int i) const { return (&v1)[i]; }
    RaveVector<T>& operator[](int i)       { return (&v1)[i]; }

    /// assumes CCW ordering of vertices 
    inline RaveVector<T> normal() {
        RaveVector<T> n;
        cross3(n, v2-v1, v3-v1);
        return n;
    }
};

/// \brief A pyramid with its vertex clipped.
/// \ingroup geometric_primitives
template <typename T>
struct frustum
{
    RaveVector<T> right, up, dir, pos;
    T fnear, ffar;
    T ffovx,ffovy;
    T fcosfovx,fsinfovx,fcosfovy,fsinfovy;
};

// Routines made for 3D graphics that deal with 3 or 4 dim algebra structures
// Functions with postfix 3 are for 3x3 operations, etc

// all fns return pfout on success or NULL on failure
// results and arguments can share pointers


////
// More complex ops that deal with arbitrary matrices //
////

/// extract eigen values and vectors from a 2x2 matrix and returns true if all values are real
/// returned eigen vectors are normalized
template <typename T>
inline bool eig2(const T* pfmat, T* peigs, T& fv1x, T& fv1y, T& fv2x, T& fv2y);

// Simple routines for linear algebra algorithms //

RAVE_API int CubicRoots (double c0, double c1, double c2, double *r0, double *r1, double *r2);
template <typename T, typename S> void Tridiagonal3 (S* mat, T* diag, T* subd);
RAVE_API bool QLAlgorithm3 (float* m_aafEntry, float* afDiag, float* afSubDiag);
RAVE_API bool QLAlgorithm3 (double* m_aafEntry, double* afDiag, double* afSubDiag);
RAVE_API void EigenSymmetric3(const double* fCovariance, double* eval, double* fAxes);

/// Computes the eigenvectors of the covariance matrix and forms a basis
/// \param[in] fCovariance a symmetric 3x3 matrix.
/// \param[out] vbasis the basis vectors extracted (form a right hand coordinate system).
template <typename T>
inline void GetCovarBasisVectors(const T fCovariance[3][3], RaveVector<T> vbasis[3])
{
    T EigenVals[3];
    T fAxes[3][3];
    EigenSymmetric3((const T*)fCovariance, EigenVals, (T*)fAxes);
    // check if we got any 0 vectors
    vbasis[0].x = fAxes[0][0];		vbasis[0].y = fAxes[1][0];		vbasis[0].z = fAxes[2][0];
    vbasis[1].x = fAxes[0][1];		vbasis[1].y = fAxes[1][1];		vbasis[1].z = fAxes[2][1];
    vbasis[2].x = fAxes[0][2];		vbasis[2].y = fAxes[1][2];		vbasis[2].z = fAxes[2][2];
    // make sure that the new axes follow the right-hand coord system
    vbasis[0].normalize3();
    vbasis[1] -= vbasis[0] * vbasis[0].dot3(vbasis[1]);
    vbasis[1].normalize3();
    vbasis[2] = vbasis[0].cross(vbasis[1]);
}

/// SVD of a 3x3 matrix A such that A = U*diag(D)*V'
/// The row stride for all matrices is 3*sizeof(T) bytes
/// \param[in] A 3x3 matrix
/// \param[out] U 3x3 matrix
/// \param[out] D 3x1 matrix
/// \param[out] V 3x3 matrix
template <typename T> inline void svd3(const T* A, T* U, T* D, T* V);

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
            costheta = dot3(v4,v5)/RaveSqrt(m1*m2);
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
    	    costheta = dot3(v4,v5)/RaveSqrt(m1*m2);
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
    cross3(vd, r.dir, vpos);
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
    vpos.x = dot3(vd, o.right);
    vdir.x = dot3(r.dir, o.right);
    if( RaveFabs(vpos.x) > o.extents.x && vdir.x* vpos.x > 0.0f) {
        return false;
    }
    vpos.y = dot3(vd, o.up);
    vdir.y = dot3(r.dir, o.up);
    if( RaveFabs(vpos.y) > o.extents.y && vdir.y * vpos.y > 0.0f) {
        return false;
    }
    vpos.z = dot3(vd, o.dir);
    vdir.z = dot3(r.dir, o.dir);
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
    vn.x = RaveFabs(dot3(vn, o.right)) - o.extents.x;
    vn.y = RaveFabs(dot3(vn, o.up)) - o.extents.y;
    vn.z = RaveFabs(dot3(vn, o.dir)) - o.extents.z;
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
    if( dot3(v,vNorm) > -o.extents.x * RaveFabs(dot3(vNorm, o.right)) -  o.extents.y * RaveFabs(dot3(vNorm, o.up)) -  o.extents.z * RaveFabs(dot3(vNorm, o.dir))) {
        return false;
    }
    vNorm = -fr.fcosfovx * fr.right - fr.fsinfovx * fr.dir;
    if(dot3(v, vNorm) > -o.extents.x * RaveFabs(dot3(vNorm, o.right)) - o.extents.y * RaveFabs(dot3(vNorm, o.up)) - o.extents.z * RaveFabs(dot3(vNorm, o.dir))) {
        return false;
    }
    vNorm = fr.fcosfovy * fr.up - fr.fsinfovy * fr.dir;
    if(dot3(v, vNorm) > -o.extents.x * RaveFabs(dot3(vNorm, o.right)) - o.extents.y * RaveFabs(dot3(vNorm, o.up)) - o.extents.z * RaveFabs(dot3(vNorm, o.dir))) {
        return false;
    }
    vNorm = -fr.fcosfovy * fr.up - fr.fsinfovy * fr.dir;
    if(dot3(v, vNorm) > -o.extents.x * RaveFabs(dot3(vNorm, o.right)) - o.extents.y * RaveFabs(dot3(vNorm, o.up)) - o.extents.z * RaveFabs(dot3(vNorm, o.dir))) {
        return false;
    }
    vNorm.x = dot3(v, fr.dir);
    vNorm.y = o.extents.x * RaveFabs(dot3(fr.dir, o.right)) +  o.extents.y * RaveFabs(dot3(fr.dir, o.up)) +  o.extents.z * RaveFabs(dot3(fr.dir, o.dir));
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
        if( dot3(o.pos,vplane)+vplane.w < o.extents.x * RaveFabs(dot3(vplane, o.right)) + o.extents.y * RaveFabs(dot3(vplane, o.up)) + o.extents.z * RaveFabs(dot3(vplane, o.dir))) {
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
    cross3(unorm, u31, u12);
    unorm.w = -dot3(unorm, u1);
    cross3(vnorm, v31, v12);
    vnorm.w = -dot3(vnorm, v1);
    if( dot3(vnorm, u1) + vnorm.w > 0 ) {
        b |= 1;
    }
    if( dot3(vnorm, u2) + vnorm.w > 0 ) {
        b |= 2;
    }
    if( dot3(vnorm, u3) + vnorm.w > 0 ) {
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
    
    T t = dot3(vnorm,*pu)+vnorm.w;
    p1 = *pu - p1 * (t / dot3(vnorm, p1));
    p2 = *pu - p2 * (t / dot3(vnorm, p2));

    // go through each of the segments in v2 and clip
    RaveVector<T> vcross;
    const RaveVector<T>* pv[] = {&v1, &v2, &v3, &v1};

    for(int i = 0; i < 3; ++i) {
        const RaveVector<T>* pprev = pv[i];
        RaveVector<T> q1 = p1 - *pprev;
        RaveVector<T> q2 = p2 - *pprev;
        cross3(vcross, vedges[i], vnorm);
        T t1 = dot3(q1, vcross);
        T t2 = dot3(q2, vcross);

        // line segment is out of face
        if( t1 >= 0 && t2 >= 0 ) {
            return false;
        }
        if( t1 > 0 && t2 < 0 ) {
            // keep second point, clip first
            RaveVector<T> dq = q2-q1;
            p1 -= dq*(t1/dot3(dq,vcross));
        }
        else if( t1 < 0 && t2 > 0 ) {
            // keep first point, clip second
            RaveVector<T> dq = q1-q2;
            p2 -= dq*(t2/dot3(dq,vcross));
        }
    }

    contactpos = 0.5f * (p1 + p2);
    normalize3(contactnorm, vnorm);
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

template <class T> inline void mult(T* pf, T fa, int r)
{
	MATH_ASSERT( pf != NULL );

	while(r > 0) {
		--r;
		pf[r] *= fa;
	}
}

template <class T> int Min(T* pts, int stride, int numPts); // returns the index, stride in units of T
template <class T> int Max(T* pts, int stride, int numPts); // returns the index

// multiplies a matrix by a scalar
template <class T> inline void mult(T* pf, T fa, int r);

// multiplies a r1xc1 by c1xc2 matrix into pfres, if badd is true adds the result to pfres
// does not handle cases where pfres is equal to pf1 or pf2, use multtox for those cases
template <class T, class R, class S>
inline S* mult(T* pf1, R* pf2, int r1, int c1, int c2, S* pfres, bool badd = false);

// pf1 is transposed before mult
// rows of pf2 must equal rows of pf1
// pfres will be c1xc2 matrix
template <class T, class R, class S>
inline S* multtrans(T* pf1, R* pf2, int r1, int c1, int c2, S* pfres, bool badd = false);

// pf2 is transposed before mult
// the columns of both matrices must be the same and equal to c1
// r2 is the number of rows in pf2
// pfres must be an r1xr2 matrix
template <class T, class R, class S>
inline S* multtrans_to2(T* pf1, R* pf2, int r1, int c1, int r2, S* pfres, bool badd = false);

// multiplies rxc matrix pf1 and cxc matrix pf2 and stores the result in pf1, 
// the function needs a temporary buffer the size of c doubles, if pftemp == NULL,
// the function will allocate the necessary memory, otherwise pftemp should be big
// enough to store all the entries
template <class T> inline T* multto1(T* pf1, T* pf2, int r1, int c1, T* pftemp = NULL);

// same as multto1 except stores the result in pf2, pf1 has to be an r2xr2 matrix
// pftemp must be of size r2 if not NULL
template <class T, class S> inline T* multto2(T* pf1, S* pf2, int r2, int c2, S* pftemp = NULL);

// add pf1 + pf2 and store in pf1
template <class T> inline void sub(T* pf1, T* pf2, int r);
template <class T> inline T normsqr(const T* pf1, int r);
template <class T> inline T lengthsqr(const T* pf1, const T* pf2, int length);
template <class T> inline T dot(T* pf1, T* pf2, int length);

template <class T> inline T sum(T* pf, int length);

/// takes the inverse of the 2x2 matrix pf and stores it into pfres, returns true if matrix is invertible
/// \ingroup affine_math
template <class T> inline bool inv2(T* pf, T* pfres);

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
        c = 1 / RaveSqrt(fv1x*fv1x + fv1y*fv1y);
        fv1x *= c;
        fv1y *= c;
        fv2x = -fv1y;
        fv2y = fv1x;
        return true;
    }
    // two roots
    d = RaveSqrt(d);
    a = -0.5f * (b + d);
    peigs[0] = a;
    fv1x = pfmat[1];
    fv1y = a-pfmat[0];
    c = 1 / RaveSqrt(fv1x*fv1x + fv1y*fv1y);
    fv1x *= c;
    fv1y *= c;
    a += d;
    peigs[1] = a;
    fv2x = pfmat[1];
    fv2y = a-pfmat[0];
    c = 1 / RaveSqrt(fv2x*fv2x + fv2y*fv2y);
    fv2x *= c;
    fv2y *= c;
    return true;
}

// returns the number of real roots, fills r1 and r2 with the answers
template <class T>
inline int solvequad(T a, T b, T c, T& r1, T& r2)
{
    T d = b * b - (T)4 * c * a + (T)1e-16;
    if( d < 0 ) {
        return 0;
    }
    if( d < (T)1e-16 ) {
        r1 = r2 = (T)-0.5 * b / a;
        return 1;
    }
    // two roots
    d = RaveSqrt(d);
    r1 = (T)-0.5 * (b + d) / a;
    r2 = r1 + d/a;
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
template <class T>
inline T* _mult3_s4(T* pfres, const T* pf1, const T* pf2)
{
    MATH_ASSERT( pf1 != NULL && pf2 != NULL && pfres != NULL );

    T* pfres2;
    if( pfres == pf1 || pfres == pf2 ) pfres2 = (T*)alloca(12 * sizeof(T));
    else pfres2 = pfres;

    MULT3(4);
    if( pfres2 != pfres ) memcpy(pfres, pfres2, 12*sizeof(T));
    return pfres;
}

/// mult3 with a 3x3 matrix whose row stride is 12 bytes
template <class T>
inline T* _mult3_s3(T* pfres, const T* pf1, const T* pf2)
{
    MATH_ASSERT( pf1 != NULL && pf2 != NULL && pfres != NULL );

    T* pfres2;
    if( pfres == pf1 || pfres == pf2 ) pfres2 = (T*)alloca(9 * sizeof(T));
    else pfres2 = pfres;

    MULT3(3);

    if( pfres2 != pfres ) memcpy(pfres, pfres2, 9*sizeof(T));

    return pfres;
}

// mult4
template <class T> 
inline T* _mult4(T* pfres, const T* p1, const T* p2)
{
    MATH_ASSERT( pfres != NULL && p1 != NULL && p2 != NULL );

    T* pfres2;
    if( pfres == p1 || pfres == p2 ) pfres2 = (T*)alloca(16 * sizeof(T));
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

template <class T> 
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

template <class T> 
inline T* _multtrans4(T* pfres, const T* pf1, const T* pf2)
{
    T* pfres2;
    if( pfres == pf1 ) pfres2 = (T*)alloca(16 * sizeof(T));
    else pfres2 = pfres;

    for(int i = 0; i < 4; ++i) {
        for(int j = 0; j < 4; ++j) {
            pfres[4*i+j] = pf1[i] * pf2[j] + pf1[i+4] * pf2[j+4] + pf1[i+8] * pf2[j+8] + pf1[i+12] * pf2[j+12];
        }
    }

    return pfres;
}

/// \brief Generate a uniformly distributed random quaternion.
/// \ingroup affine_math
template <typename T>
inline RaveVector<T> GetRandomQuat()
{
    RaveVector<T> q;
    while(1) {
        q.x = -1 + 2*(T)(MATH_RANDOM_FLOAT);
        q.y = -1 + 2*(T)(MATH_RANDOM_FLOAT);
        q.z = -1 + 2*(T)(MATH_RANDOM_FLOAT);
        q.w = -1 + 2*(T)(MATH_RANDOM_FLOAT);
        T norm = q.lengthsqr4();
        if(norm <= 1) {
            q = q * (1 / RaveSqrt(norm));
            break;
        }
    }
    return q;
}

/// \brief Convertex axis angle reprensetation to quaternions.
/// \ingroup affine_math
template <class T>
inline RaveVector<T> AxisAngle2Quat(const RaveVector<T>& rotaxis, T angle)
{
    angle *= (T)0.5;
    T fsin = RaveSin(angle);
    return RaveVector<T>(RaveCos(angle), rotaxis.x*fsin, rotaxis.y * fsin, rotaxis.z * fsin);
}

/// \brief Multiply two quaternions
/// \ingroup affine_math
template <class T>
inline RaveVector<T> quatMultiply(const RaveVector<T>& q0, const RaveVector<T>& q1)
{
    RaveVector<T> q(q0.x*q1.x - q0.y*q1.y - q0.z*q1.z - q0.w*q1.w,
                    q0.x*q1.y + q0.y*q1.x + q0.z*q1.w - q0.w*q1.z,
                    q0.x*q1.z + q0.z*q1.x + q0.w*q1.y - q0.y*q1.w,
                    q0.x*q1.w + q0.w*q1.x + q0.y*q1.z - q0.z*q1.y);
    // normalize the quaternion
    T fnorm = q.lengthsqr4();
    MATH_ASSERT( fnorm > 0.98f && fnorm < 1.02f );
    return q * (T(1)/RaveSqrt(fnorm));
}

/// \brief Inverted a quaternion rotation.
/// \ingroup affine_math
template <class T>
inline RaveVector<T> quatInverse(const RaveVector<T>& q)
{
    return RaveVector<T>(q.x,-q.y,-q.z,-q.w);
}

/// \brief Quaternion spherical linear interpolation.
/// \ingroup affine_math
template <class T>
inline RaveVector<T> dQSlerp(const RaveVector<T>& qa, const RaveVector<T>& _qb, T t)
{
    // quaternion to return
    RaveVector<T> qb, qm;
    if( dot4(qa,_qb) < 0 ) {
        qb = -_qb;
    }
    else {
        qb = _qb;
    }
    // Calculate angle between them.
    T cosHalfTheta = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z;
    // if qa=qb or qa=-qb then theta = 0 and we can return qa
    if (RaveFabs(cosHalfTheta) >= 1.0){
        qm.w = qa.w;qm.x = qa.x;qm.y = qa.y;qm.z = qa.z;
        return qm;
    }
    // Calculate temporary values.
    T halfTheta = RaveAcos(cosHalfTheta);
    T sinHalfTheta = RaveSqrt(1 - cosHalfTheta*cosHalfTheta);
    // if theta = 180 degrees then result is not fully defined
    // we could rotate around any axis normal to qa or qb
    if (RaveFabs(sinHalfTheta) < 1e-7f){ // fabs is floating point absolute
        qm.w = (qa.w * 0.5f + qb.w * 0.5f);
        qm.x = (qa.x * 0.5f + qb.x * 0.5f);
        qm.y = (qa.y * 0.5f + qb.y * 0.5f);
        qm.z = (qa.z * 0.5f + qb.z * 0.5f);
        return qm;
    }

    T ratioA = RaveSin((1 - t) * halfTheta) / sinHalfTheta;
    T ratioB = RaveSin(t * halfTheta) / sinHalfTheta; 
    //calculate Quaternion.
    qm.w = (qa.w * ratioA + qb.w * ratioB);
    qm.x = (qa.x * ratioA + qb.x * ratioB);
    qm.y = (qa.y * ratioA + qb.y * ratioB);
    qm.z = (qa.z * ratioA + qb.z * ratioB);
    return qm;
}

/// \brief Return the minimal quaternion that orients vsource to vtarget.
/// \ingroup affine_math
template<typename T>
RaveVector<T> quatRotateDirection(const RaveVector<T>& vsource, const RaveVector<T>& vtarget)
{
    RaveVector<T> rottodirection;
    cross3(rottodirection, vsource,vtarget);
    T fsin = RaveSqrt(rottodirection.lengthsqr3());
    T fcos = dot3(vsource, vtarget);
    RaveTransform<T> torient;
    if( fsin > 1e-6f ) {
        torient.rotfromaxisangle(rottodirection*(1/fsin), RaveAtan2(fsin, fcos));
    }
    else if( fcos < 0 ) {
        // hand is flipped 180, rotate around x axis
        rottodirection = RaveVector<T>(1,0,0);
        rottodirection -= vsource * dot3(vsource, rottodirection);
        if( rottodirection.lengthsqr3()<1e-8 ) {
            rottodirection = RaveVector<T>(0,0,1);
            rottodirection -= vsource * dot3(vsource, rottodirection);
        }
        rottodirection.normalize3();
        torient.rotfromaxisangle(rottodirection, RaveAtan2(fsin, fcos));
    }
    return torient.rot;
}

/// \brief Returns a camera matrix that looks along a ray with a desired up vector.
///
/// \ingroup affine_math
/// \param lookat the point space to look at, the camera will rotation and zoom around this point
/// \param campos the position of the camera in space
/// \param camup vector from the camera
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

/// \brief Compute the determinant of a 3x3 matrix whose row stride stride elements.
/// \ingroup affine_math
template <class T> inline T matrixdet3(const T* pf, int stride)
{
    return pf[0*stride+2] * (pf[1*stride + 0] * pf[2*stride + 1] - pf[1*stride + 1] * pf[2*stride + 0]) +
        pf[1*stride+2] * (pf[0*stride + 1] * pf[2*stride + 0] - pf[0*stride + 0] * pf[2*stride + 1]) +
        pf[2*stride+2] * (pf[0*stride + 0] * pf[1*stride + 1] - pf[0*stride + 1] * pf[1*stride + 0]);
}

/** \brief 3x3 matrix inverse.

    \ingroup affine_math
    \param[in] pf the input 3x3 matrix
    \param[out] pf the result of the operation, can be the same matrix as pf
    \param[out] pfdet if not NULL, fills it with the determinant of the source matrix
    \param[in] stride the stride in elements between elements.
*/
template <class T>
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
		pfres[0*stride+0] *= fdet;		pfres[0*stride+1] *= fdet;		pfres[0*stride+2] *= fdet;
		pfres[1*stride+0] *= fdet;		pfres[1*stride+1] *= fdet;		pfres[1*stride+2] *= fdet;
		pfres[2*stride+0] *= fdet;		pfres[2*stride+1] *= fdet;		pfres[2*stride+2] *= fdet;
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
/// \ingroup affine_math
template <class T>
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

	pfres2[2*4 + 0] =   pf[1*4 + 0] * f1 -	pf[1*4 + 1] * f2 + pf[1*4 + 3] * fd0;
	pfres2[2*4 + 1] = -(pf[0*4 + 0] * f1 -	pf[0*4 + 1] * f2 + pf[0*4 + 3] * fd0);

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

	pfres2[2*4 + 2] =   pf[3*4 + 0] * f1 -	pf[3*4 + 1] * f2 + pf[3*4 + 3] * fd0;
	pfres2[2*4 + 3] = -(pf[2*4 + 0] * f1 -	pf[2*4 + 1] * f2 + pf[2*4 + 3] * fd0);

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
/// \ingroup affine_math
template <class T>
inline T* _transpose3(const T* pf, T* pfres)
{
	MATH_ASSERT( pf != NULL && pfres != NULL );

	if( pf == pfres ) {
        std::swap(pfres[1], pfres[3]);
		std::swap(pfres[2], pfres[6]);
		std::swap(pfres[5], pfres[7]);
		return pfres;
	}

	pfres[0] = pf[0];	pfres[1] = pf[3];	pfres[2] = pf[6];
	pfres[3] = pf[1];	pfres[4] = pf[4];	pfres[5] = pf[7];
	pfres[6] = pf[2];	pfres[7] = pf[5];	pfres[8] = pf[8];

	return pfres;
}

/// \brief Transpose a 4x4 matrix.
/// \ingroup affine_math
template <class T>
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

	pfres[0] = pf[0];	pfres[1] = pf[4];	pfres[2] = pf[8];		pfres[3] = pf[12];
	pfres[4] = pf[1];	pfres[5] = pf[5];	pfres[6] = pf[9];		pfres[7] = pf[13];
	pfres[8] = pf[2];	pfres[9] = pf[6];	pfres[10] = pf[10];		pfres[11] = pf[14];
	pfres[12] = pf[3];	pfres[13] = pf[7];	pfres[14] = pf[11];		pfres[15] = pf[15];
	return pfres;
}

template <class T>
inline T _dot2(const T* pf1, const T* pf2)
{
	MATH_ASSERT( pf1 != NULL && pf2 != NULL );
	return pf1[0]*pf2[0] + pf1[1]*pf2[1];
}

template <class T>
inline T _dot3(const T* pf1, const T* pf2)
{
	MATH_ASSERT( pf1 != NULL && pf2 != NULL );
	return pf1[0]*pf2[0] + pf1[1]*pf2[1] + pf1[2]*pf2[2];
}

template <class T>
inline T _dot4(const T* pf1, const T* pf2)
{
	MATH_ASSERT( pf1 != NULL && pf2 != NULL );
	return pf1[0]*pf2[0] + pf1[1]*pf2[1] + pf1[2]*pf2[2] + pf1[3] * pf2[3];
}

template <class T>
inline T _lengthsqr2(const T* pf)
{
	MATH_ASSERT( pf != NULL );
	return pf[0] * pf[0] + pf[1] * pf[1];
}

template <class T>
inline T _lengthsqr3(const T* pf)
{
	MATH_ASSERT( pf != NULL );
	return pf[0] * pf[0] + pf[1] * pf[1] + pf[2] * pf[2];
}

template <class T>
inline T _lengthsqr4(const T* pf)
{
	MATH_ASSERT( pf != NULL );
	return pf[0] * pf[0] + pf[1] * pf[1] + pf[2] * pf[2] + pf[3] * pf[3];
}

template <class T>
inline T* _normalize2(T* pfout, const T* pf)
{
	MATH_ASSERT(pf != NULL);

	T f = pf[0]*pf[0] + pf[1]*pf[1];
	f = 1 / RaveSqrt(f);
	pfout[0] = pf[0] * f;
	pfout[1] = pf[1] * f;

	return pfout;
}

template <class T>
inline T* _normalize3(T* pfout, const T* pf)
{
	MATH_ASSERT(pf != NULL);

	T f = pf[0]*pf[0] + pf[1]*pf[1] + pf[2]*pf[2];

	f = 1 / RaveSqrt(f);
	pfout[0] = pf[0] * f;
	pfout[1] = pf[1] * f;
	pfout[2] = pf[2] * f;

	return pfout;
}

template <class T>
inline T* _normalize4(T* pfout, const T* pf)
{
	MATH_ASSERT(pf != NULL);

	T f = pf[0]*pf[0] + pf[1]*pf[1] + pf[2]*pf[2] + pf[3]*pf[3];

	f = 1 / RaveSqrt(f);
	pfout[0] = pf[0] * f;
	pfout[1] = pf[1] * f;
	pfout[2] = pf[2] * f;
	pfout[3] = pf[3] * f;

	return pfout;
}

template <class T>
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

template <class T>
inline T* transcoord3(T* pfout, const RaveTransformMatrix<T>* pmat, const T* pf)
{
	MATH_ASSERT( pfout != NULL && pf != NULL && pmat != NULL );
    
    T dummy[3];
	T* pfreal = (pfout == pf) ? dummy : pfout;

	pfreal[0] = pf[0] * pmat->m[0] + pf[1] * pmat->m[1] + pf[2] * pmat->m[2] + pmat->trans.x;
    pfreal[1] = pf[0] * pmat->m[4] + pf[1] * pmat->m[5] + pf[2] * pmat->m[6] + pmat->trans.y;
    pfreal[2] = pf[0] * pmat->m[8] + pf[1] * pmat->m[9] + pf[2] * pmat->m[10] + pmat->trans.z;

    if( pfout ==pf ) {
        pfout[0] = pfreal[0];
        pfout[1] = pfreal[1];
        pfout[2] = pfreal[2];
    }

	return pfout;
}

template <class T>
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

template <class T>
inline T* transnorm3(T* pfout, const RaveTransformMatrix<T>* pmat, const T* pf)
{
	MATH_ASSERT( pfout != NULL && pf != NULL && pmat != NULL );

	T dummy[3];
    T* pfreal = (pfout == pf) ? dummy : pfout;

	pfreal[0] = pf[0] * pmat->m[0] + pf[1] * pmat->m[1] + pf[2] * pmat->m[2];
    pfreal[1] = pf[0] * pmat->m[4] + pf[1] * pmat->m[5] + pf[2] * pmat->m[6];
    pfreal[2] = pf[0] * pmat->m[8] + pf[1] * pmat->m[9] + pf[2] * pmat->m[10];

	if( pfreal != pfout ) {
        pfout[0] = pfreal[0];
        pfout[1] = pfreal[1];
        pfout[2] = pfreal[2];
    }

	return pfout;
}

inline float* mult4(float* pfres, const float* pf1, const float* pf2) { return _mult4<float>(pfres, pf1, pf2); }
// pf1^T * pf2
inline float* multtrans3(float* pfres, const float* pf1, const float* pf2) { return _multtrans3<float>(pfres, pf1, pf2); }
inline float* multtrans4(float* pfres, const float* pf1, const float* pf2) { return _multtrans4<float>(pfres, pf1, pf2); }
inline float* transnorm3(float* pfout, const float* pfmat, const float* pf) { return _transnorm3<float>(pfout, pfmat, pf); }

inline float* transpose3(const float* pf, float* pfres) { return _transpose3<float>(pf, pfres); }
inline float* transpose4(const float* pf, float* pfres) { return _transpose4<float>(pf, pfres); }

inline float dot2(const float* pf1, const float* pf2) { return _dot2<float>(pf1, pf2); }
inline float dot3(const float* pf1, const float* pf2) { return _dot3<float>(pf1, pf2); }
inline float dot4(const float* pf1, const float* pf2) { return _dot4<float>(pf1, pf2); }

inline float lengthsqr2(const float* pf) { return _lengthsqr2<float>(pf); }
inline float lengthsqr3(const float* pf) { return _lengthsqr3<float>(pf); }
inline float lengthsqr4(const float* pf) { return _lengthsqr4<float>(pf); }

inline float* normalize2(float* pfout, const float* pf) { return _normalize2<float>(pfout, pf); }
inline float* normalize3(float* pfout, const float* pf) { return _normalize3<float>(pfout, pf); }
inline float* normalize4(float* pfout, const float* pf) { return _normalize4<float>(pfout, pf); }

inline float* cross3(float* pfout, const float* pf1, const float* pf2) { return _cross3<float>(pfout, pf1, pf2); }

// multiplies 3x3 matrices
inline float* mult3_s4(float* pfres, const float* pf1, const float* pf2) { return _mult3_s4<float>(pfres, pf1, pf2); }
inline float* mult3_s3(float* pfres, const float* pf1, const float* pf2) { return _mult3_s3<float>(pfres, pf1, pf2); }

inline float* inv3(const float* pf, float* pfres, float* pfdet, int stride) { return _inv3<float>(pf, pfres, pfdet, stride); }
inline float* inv4(const float* pf, float* pfres) { return _inv4<float>(pf, pfres); }


inline double* mult4(double* pfres, const double* pf1, const double* pf2) { return _mult4<double>(pfres, pf1, pf2); }
// pf1^T * pf2
inline double* multtrans3(double* pfres, const double* pf1, const double* pf2) { return _multtrans3<double>(pfres, pf1, pf2); }
inline double* multtrans4(double* pfres, const double* pf1, const double* pf2) { return _multtrans4<double>(pfres, pf1, pf2); }
inline double* transnorm3(double* pfout, const double* pfmat, const double* pf) { return _transnorm3<double>(pfout, pfmat, pf); }

inline double* transpose3(const double* pf, double* pfres) { return _transpose3<double>(pf, pfres); }
inline double* transpose4(const double* pf, double* pfres) { return _transpose4<double>(pf, pfres); }

inline double dot2(const double* pf1, const double* pf2) { return _dot2<double>(pf1, pf2); }
inline double dot3(const double* pf1, const double* pf2) { return _dot3<double>(pf1, pf2); }
inline double dot4(const double* pf1, const double* pf2) { return _dot4<double>(pf1, pf2); }

inline double lengthsqr2(const double* pf) { return _lengthsqr2<double>(pf); }
inline double lengthsqr3(const double* pf) { return _lengthsqr3<double>(pf); }
inline double lengthsqr4(const double* pf) { return _lengthsqr4<double>(pf); }

inline double* normalize2(double* pfout, const double* pf) { return _normalize2<double>(pfout, pf); }
inline double* normalize3(double* pfout, const double* pf) { return _normalize3<double>(pfout, pf); }
inline double* normalize4(double* pfout, const double* pf) { return _normalize4<double>(pfout, pf); }

inline double* cross3(double* pfout, const double* pf1, const double* pf2) { return _cross3<double>(pfout, pf1, pf2); }

// multiplies 3x3 matrices
inline double* mult3_s4(double* pfres, const double* pf1, const double* pf2) { return _mult3_s4<double>(pfres, pf1, pf2); }
inline double* mult3_s3(double* pfres, const double* pf1, const double* pf2) { return _mult3_s3<double>(pfres, pf1, pf2); }

inline double* inv3(const double* pf, double* pfres, double* pfdet, int stride) { return _inv3<double>(pf, pfres, pfdet, stride); }
inline double* inv4(const double* pf, double* pfres) { return _inv4<double>(pf, pfres); }

template <class T, class R, class S>
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

template <class T, class R, class S>
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

template <class T, class R, class S>
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

template <class T> inline T* multto1(T* pf1, T* pf2, int r, int c, T* pftemp)
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

template <class T, class S> inline T* multto2(T* pf1, S* pf2, int r2, int c2, S* pftemp)
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

template <class T> inline void add(T* pf1, T* pf2, int r)
{
	MATH_ASSERT( pf1 != NULL && pf2 != NULL);

	while(r > 0) {
		--r;
		pf1[r] += pf2[r];
	}
}

template <class T> inline void sub(T* pf1, T* pf2, int r)
{
	MATH_ASSERT( pf1 != NULL && pf2 != NULL);

	while(r > 0) {
		--r;
		pf1[r] -= pf2[r];
	}
}

template <class T> inline T normsqr(const T* pf1, int r)
{
	MATH_ASSERT( pf1 != NULL );

	T d = 0.0;
	while(r > 0) {
		--r;
		d += pf1[r] * pf1[r];
	}

	return d;
}

template <class T> inline T lengthsqr(const T* pf1, const T* pf2, int length)
{
	T d = 0;
	while(length > 0) {
		--length;
        T t = pf1[length] - pf2[length];
		d += t * t;
	}

	return d;
}

template <class T> inline T dot(T* pf1, T* pf2, int length)
{
	T d = 0;
	while(length > 0) {
		--length;
		d += pf1[length] * pf2[length];
	}

	return d;
}

template <class T> inline T sum(T* pf, int length)
{
	T d = 0;
	while(length > 0) {
		--length;
		d += pf[length];
	}

	return d;
}

template <class T> inline bool inv2(T* pf, T* pfres)
{
	T fdet = pf[0] * pf[3] - pf[1] * pf[2];

	if( fabs(fdet) < 1e-16 ) return false;

	fdet = 1 / fdet;
	//if( pfdet != NULL ) *pfdet = fdet;

	if( pfres != pf ) {
		pfres[0] = fdet * pf[3];		pfres[1] = -fdet * pf[1];
		pfres[2] = -fdet * pf[2];		pfres[3] = fdet * pf[0];
		return true;
	}

	T ftemp = pf[0];
	pfres[0] = pf[3] * fdet;
	pfres[1] *= -fdet;
	pfres[2] *= -fdet;
	pfres[3] = ftemp * pf[0];

	return true;
}

template <class T, class S>
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
        ell = (T)RaveSqrt(b*b+c*c);
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
        D[i] = RaveSqrt(eigenvalues[i]);
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

template <class T>
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

template <class T>
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

// Don't add new lines to the output << operators. Some applications use it to serialize the data
// to send across the network.

template <class T, class U>
std::basic_ostream<T>& operator<<(std::basic_ostream<T>& O, const RaveVector<U>& v)
{
    return O << v.x << " " << v.y << " " << v.z << " " << v.w << " ";
}

template <class T, class U>
std::basic_istream<T>& operator>>(std::basic_istream<T>& I, RaveVector<U>& v)
{
    return I >> v.x >> v.y >> v.z >> v.w;
}

template <class T, class U>
std::basic_ostream<T>& operator<<(std::basic_ostream<T>& O, const RaveTransform<U>& v)
{
    return O << v.rot.x << " " << v.rot.y << " " << v.rot.z << " " << v.rot.w << " "
             << v.trans.x << " " << v.trans.y << " " << v.trans.z << " ";
}

template <class T, class U>
std::basic_istream<T>& operator>>(std::basic_istream<T>& I, RaveTransform<U>& v)
{
    return I >> v.rot.x >> v.rot.y >> v.rot.z >> v.rot.w >> v.trans.x >> v.trans.y >> v.trans.z;
}

// serial in column order! This is the format transformations are passed across the network
template <class T, class U>
std::basic_ostream<T>& operator<<(std::basic_ostream<T>& O, const RaveTransformMatrix<U>& v)
{
    return O << v.m[0] << " " << v.m[4] << " " << v.m[8] << " "
             << v.m[1] << " " << v.m[5] << " " << v.m[9] << " "
             << v.m[2] << " " << v.m[6] << " " << v.m[10] << " "
             << v.trans.x << " " << v.trans.y << " " << v.trans.z << " ";
}

// read in column order! This is the format transformations are passed across the network
template <class T, class U>
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
