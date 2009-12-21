// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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
#include "libopenrave.h"

#include <algorithm>

namespace OpenRAVE {

// code from MagicSoftware by Dave Eberly

//===========================================================================

int CubicRoots (double c0, double c1, double c2, double *r0, double *r1, double *r2)
{
    // polynomial is L^3-c2*L^2+c1*L-c0

    int maxiter = 50;
    double discr, temp, pval, pdval, b0, b1;
    int i;

    // find local extrema (if any) of p'(L) = 3*L^2-2*c2*L+c1
    discr = c2*c2-3*c1;
    if ( discr >= 0.0 ) {
        discr = (double)sqrt(discr);
        temp = (c2+discr)/3;
        pval = temp*(temp*(temp-c2)+c1)-c0;
        if ( pval >= 0.0 ) {
            // double root occurs before the positive local maximum
            (*r0) = (c2-discr)/3 - 1;  // initial guess for Newton's methods
            pval = 2*g_fEpsilon;
            for (i = 0; i < maxiter && fabs(pval) > g_fEpsilon; i++) {
                pval = (*r0)*((*r0)*((*r0)-c2)+c1)-c0;
                pdval = (*r0)*(3*(*r0)-2*c2)+c1;
                (*r0) -= pval/pdval;
            }

            // Other two roots are solutions to quadratic equation
            // L^2 + ((*r0)-c2)*L + [(*r0)*((*r0)-c2)+c1] = 0.
            b1 = (*r0)-c2;
            b0 = (*r0)*((*r0)-c2)+c1;
            discr = b1*b1-4*b0;
            if ( discr < -g_fEpsilon )
            {
                // single root r0
                return singleRoot;
            }
            else
            {
                int result = distinctRoots;

                // roots r0 <= r1 <= r2
                discr = sqrt(fabs(discr));
                (*r1) = 0.5f*(-b1-discr);
                (*r2) = 0.5f*(-b1+discr);

                if ( fabs((*r0)-(*r1)) <= g_fEpsilon )
                {
                    (*r0) = (*r1);
                    result |= floatRoot01;
                }
                if ( fabs((*r1)-(*r2)) <= g_fEpsilon )
                {
                    (*r1) = (*r2);
                    result |= floatRoot12;
                }
                return result;
            }
        }
        else {
            // double root occurs after the negative local minimum
            (*r2) = temp + 1;  // initial guess for Newton's method
            pval = 2*g_fEpsilon;
            for (i = 0; i < maxiter && fabs(pval) > g_fEpsilon; i++) {
                pval = (*r2)*((*r2)*((*r2)-c2)+c1)-c0;
                pdval = (*r2)*(3*(*r2)-2*c2)+c1;
                (*r2) -= pval/pdval;
            }

            // Other two roots are solutions to quadratic equation
            // L^2 + (r2-c2)*L + [r2*(r2-c2)+c1] = 0.
            b1 = (*r2)-c2;
            b0 = (*r2)*((*r2)-c2)+c1;
            discr = b1*b1-4*b0;
            if ( discr < -g_fEpsilon )
            {
                // single root
                (*r0) = (*r2);
                return singleRoot;
            }
            else
            {
                int result = distinctRoots;

                // roots r0 <= r1 <= r2
                discr = sqrt(fabs(discr));
                (*r0) = 0.5f*(-b1-discr);
                (*r1) = 0.5f*(-b1+discr);

                if ( fabs((*r0)-(*r1)) <= g_fEpsilon )
                {
                    (*r0) = (*r1);
                    result |= floatRoot01;
                }
                if ( fabs((*r1)-(*r2)) <= g_fEpsilon )
                {
                    (*r1) = (*r2);
                    result |= floatRoot12;
                }
                return result;
            }
        }
    }
    else {
        // p(L) has one double root
        (*r0) = c0;
        pval = 2*g_fEpsilon;
        for (i = 0; i < maxiter && fabs(pval) > g_fEpsilon; i++) {
            pval = (*r0)*((*r0)*((*r0)-c2)+c1)-c0;
            pdval = (*r0)*(3*(*r0)-2*c2)+c1;
            (*r0) -= pval/pdval;
        }
        return singleRoot;
    }
}

//----------------------------------------------------------------------------
template <class T>
bool _QLAlgorithm3 (T* m_aafEntry, T* afDiag, T* afSubDiag)
{
    // QL iteration with implicit shifting to reduce matrix from tridiagonal
    // to diagonal

    for (int i0 = 0; i0 < 3; i0++)
    {
        const int iMaxIter = 32;
        int iIter;
        for (iIter = 0; iIter < iMaxIter; iIter++)
        {
            int i1;
            for (i1 = i0; i1 <= 1; i1++)
            {
                T fSum = RaveFabs(afDiag[i1]) +
                    RaveFabs(afDiag[i1+1]);
                if ( RaveFabs(afSubDiag[i1]) + fSum == fSum )
                    break;
            }
            if ( i1 == i0 )
                break;

            T fTmp0 = (afDiag[i0+1]-afDiag[i0])/(2.0f*afSubDiag[i0]);
            T fTmp1 = RaveSqrt(fTmp0*fTmp0+1.0f);
            if ( fTmp0 < 0.0f )
                fTmp0 = afDiag[i1]-afDiag[i0]+afSubDiag[i0]/(fTmp0-fTmp1);
            else
                fTmp0 = afDiag[i1]-afDiag[i0]+afSubDiag[i0]/(fTmp0+fTmp1);
            T fSin = 1.0f;
            T fCos = 1.0f;
            T fTmp2 = 0.0f;
            for (int i2 = i1-1; i2 >= i0; i2--)
            {
                 T fTmp3 = fSin*afSubDiag[i2];
                T fTmp4 = fCos*afSubDiag[i2];
                if ( RaveFabs(fTmp3) >= RaveFabs(fTmp0) )
                {
                    fCos = fTmp0/fTmp3;
                    fTmp1 = RaveSqrt(fCos*fCos+1.0f);
                    afSubDiag[i2+1] = fTmp3*fTmp1;
                    fSin = 1.0f/fTmp1;
                    fCos *= fSin;
                }
                else
                {
                    fSin = fTmp3/fTmp0;
                    fTmp1 = RaveSqrt(fSin*fSin+1.0f);
                    afSubDiag[i2+1] = fTmp0*fTmp1;
                    fCos = 1.0f/fTmp1;
                    fSin *= fCos;
                }
                fTmp0 = afDiag[i2+1]-fTmp2;
                fTmp1 = (afDiag[i2]-fTmp0)*fSin+2.0f*fTmp4*fCos;
                fTmp2 = fSin*fTmp1;
                afDiag[i2+1] = fTmp0+fTmp2;
                fTmp0 = fCos*fTmp1-fTmp4;

                for (int iRow = 0; iRow < 3; iRow++)
                {
                    fTmp3 = m_aafEntry[iRow*3+i2+1];
                    m_aafEntry[iRow*3+i2+1] = fSin*m_aafEntry[iRow*3+i2] +
                        fCos*fTmp3;
                    m_aafEntry[iRow*3+i2] = fCos*m_aafEntry[iRow*3+i2] -
                        fSin*fTmp3;
                }
            }
            afDiag[i0] -= fTmp2;
            afSubDiag[i0] = fTmp0;
            afSubDiag[i1] = 0.0f;
        }

        if ( iIter == iMaxIter )
        {
            // should not get here under normal circumstances
            return false;
        }
    }

    return true;
}

bool QLAlgorithm3 (float* m_aafEntry, float* afDiag, float* afSubDiag)
{
    return _QLAlgorithm3<float>(m_aafEntry, afDiag, afSubDiag);
}

bool QLAlgorithm3 (double* m_aafEntry, double* afDiag, double* afSubDiag)
{
    return _QLAlgorithm3<double>(m_aafEntry, afDiag, afSubDiag);
}


void EigenSymmetric3(float* fmat, float* afEigenvalue, float* fevecs)
{
    float afSubDiag[3];

	memcpy(fevecs, fmat, sizeof(float)*9);
    Tridiagonal3(fevecs, afEigenvalue,afSubDiag);
    QLAlgorithm3(fevecs, afEigenvalue,afSubDiag);

    // make eigenvectors form a right--handed system
    float fDet =	fevecs[0*3+0] * (fevecs[1*3+1] * fevecs[2*3+2] - fevecs[1*3+2] * fevecs[2*3+1]) +
					fevecs[0*3+1] * (fevecs[1*3+2] * fevecs[2*3+0] - fevecs[1*3+0] * fevecs[2*3+2]) +
					fevecs[0*3+2] * (fevecs[1*3+0] * fevecs[2*3+1] - fevecs[1*3+1] * fevecs[2*3+0]);
    if ( fDet < 0.0f )
    {
        fevecs[0*3+2] = - fevecs[0*3+2];
        fevecs[1*3+2] = - fevecs[1*3+2];
        fevecs[2*3+2] = - fevecs[2*3+2];
    }
}
/* end of MAGIC code */

void svd3(const dReal* A, dReal* U, dReal* D, dReal* V)
{
    dReal VVt[9];
    dReal eigenvalues[3];

    multtrans3(VVt, A, A);

    // get eigen values of V: VVt  V = V  D^2
    dReal afSubDiag[3];
	memcpy(V, VVt, sizeof(dReal)*9);
    Tridiagonal3(V, eigenvalues,afSubDiag);
    QLAlgorithm3(V, eigenvalues,afSubDiag);

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
        D[i] = sqrtf(eigenvalues[i]);
        float f = 1/D[i];
        U[i] *= f;
        U[i+3] *= f;
        U[i+6] *= f;
    }

    int maxval = 0;
    if( D[1] > D[maxval] ) maxval = 1;
    if( D[2] > D[maxval] ) maxval = 2;

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

void GetCovarBasisVectors(float fCovariance[3][3], Vector* vRight, Vector* vUp, Vector* vDir)
{
	float EigenVals[3];
	float fAxes[3][3];

	EigenSymmetric3((float*)fCovariance, EigenVals, (float*)fAxes);

	// check if we got any 0 vectors
	vRight->x = fAxes[0][0];		vRight->y = fAxes[1][0];		vRight->z = fAxes[2][0];
	vUp->x = fAxes[0][1];		vUp->y = fAxes[1][1];		vUp->z = fAxes[2][1];
	vDir->x = fAxes[0][2];		vDir->y = fAxes[1][2];		vDir->z = fAxes[2][2];

	// make sure that the new axes follow the left-hand coord system
	normalize3(&vRight->x, &vRight->x);
	*vUp -= *vRight * dot3(&vUp->x, &vRight->x);
	normalize3(&vUp->x, &vUp->x);

	cross3(&vDir->x, &vRight->x, &vUp->x);
}

void QuadraticSolver(float* pfq, float* pfroots)
{
	dReal d = pfq[1] * pfq[1] - 4 * pfq[0] * pfq[2];

	if( d < 1e-10 ) {
		pfroots[0] = pfroots[1] = 0;
	}

	d = sqrtf(d);
	dReal a = (dReal)0.5 / pfq[0];
	pfroots[0] = -(pfq[1] + d) * a;
	pfroots[1] = -(pfq[1] - d) * a;
}

int insideQuadrilateral(const Vector* v, const Vector* v0,const Vector* v1, const Vector* v2,const Vector* v3) 
{
#define EPSILON  0.00001

	Vector verts[4];
	verts[0] = *v0;
	verts[1] = *v1;
	verts[2] = *v2;
	verts[3] = *v3; 
	
	Vector v4,v5;
  
    dReal m1,m2;
	dReal anglesum=0,costheta;

	for (int i=0;i<4;i++) {
  		v4.x = verts[i].x - v->x;
  		v4.y = verts[i].y - v->y;
  		v4.z = verts[i].z - v->z;
  		v5.x = verts[(i+1)%4].x - v->x;
  		v5.y = verts[(i+1)%4].y - v->y;
  		v5.z = verts[(i+1)%4].z - v->z;

  		m1 = v4.lengthsqr3();
  		m2 = v5.lengthsqr3();
  		if (m1*m2 <= EPSILON)
    		return(1); /* We are on a node, consider this inside */
  		else
    		costheta = (v4.x*v5.x + v4.y*v5.y + v4.z*v5.z) / sqrtf(m1*m2);

  		anglesum += RaveAcos(costheta);
	}
	
	dReal diff = anglesum - (dReal)2.0 * PI;

	if (diff*diff <= EPSILON*EPSILON) return 1;

	return 0;
}

int insideTriangle(const Vector* v, const Vector* v0, const Vector* v1, const Vector* v2) 
{
#define EPSILON  0.00001

	Vector verts[3];
	verts[0] = *v0;
	verts[1] = *v1;
	verts[2] = *v2; 
	
	Vector v4,v5;
  
    dReal m1,m2;
    dReal anglesum=0.0;
	dReal costheta;

	for (int i=0;i<3;i++) {

  	    v4.x = verts[i].x - v->x;
  	    v4.y = verts[i].y - v->y;
  	    v4.z = verts[i].z - v->z;
  	    v5.x = verts[(i+1)%3].x - v->x;
  	    v5.y = verts[(i+1)%3].y - v->y;
  	    v5.z = verts[(i+1)%3].z - v->z;

  	    m1 = v4.lengthsqr3();
  	    m2 = v5.lengthsqr3();
  	    if (m1*m2 <= EPSILON)
    	    return(1); /* We are on a node, consider this inside */
  	    else
    	    costheta = (v4.x*v5.x + v4.y*v5.y + v4.z*v5.z) / sqrtf(m1*m2);

  	    anglesum += acos(costheta);
	}
	float diff = anglesum - (dReal)2.0 * PI;
	if (sqrtf(diff*diff) <= EPSILON) return (1);

	return (0);
}

bool RayOBBTest(const RAY& r, const OBB& o)
{
    Vector vpos, vdir, vd;
	vd = r.pos - o.pos;		// temporary

	vpos.x = dot3(vd, o.right);
	vdir.x = dot3(r.dir, o.right);
	if( fabsf(vpos.x) > o.extents.x && vdir.x* vpos.x > 0.0f) return false;

	vpos.y = dot3(vd, o.up);
	vdir.y = dot3(r.dir, o.up);
	if( fabsf(vpos.y) > o.extents.y && vdir.y * vpos.y > 0.0f) return false;

	vpos.z = dot3(vd, o.dir);
	vdir.z = dot3(r.dir, o.dir);
	if( fabsf(vpos.z) > o.extents.z && vdir.z * vpos.z > 0.0f) return false;

	cross3(vd, vdir, vpos);

	if( fabsf(vd.x) > o.extents.y * fabsf(vdir.z) + o.extents.z * fabsf(vdir.y) ) return false;
	if( fabsf(vd.y) > o.extents.x * fabsf(vdir.z) + o.extents.z * fabsf(vdir.x) ) return false;
	if( fabsf(vd.z) > o.extents.x * fabsf(vdir.y) + o.extents.y * fabsf(vdir.x) ) return false;

	return true;
}

// the minimum distance form the vertex to the obb
dReal DistVertexOBBSq(const Vector& v, const OBB& o)
{
	Vector vn = v - o.pos;
	vn.x = fabs(dot3(vn, o.right)) - o.extents.x;
    vn.y = fabs(dot3(vn, o.up)) - o.extents.y;
    vn.z = fabs(dot3(vn, o.dir)) - o.extents.z;

	// now we have the vertex in OBB's frame
	dReal fDist = 0;

	if( vn.x > 0.0f ) fDist += vn.x * vn.x;
	if( vn.y > 0.0f ) fDist += vn.y * vn.y;
	if( vn.z > 0.0f ) fDist += vn.z * vn.z;

	return fDist;
}

//CONTACT NORMAL IS THE NORMAL OF THE SECOND TRIANGLE
template <class T>
bool TriTriCollision_temp(const RaveVector<T>& u1, const RaveVector<T>& u2, const RaveVector<T>& u3,
                   const RaveVector<T>& v1, const RaveVector<T>& v2, const RaveVector<T>& v3,
                     RaveVector<T>& contactpos, RaveVector<T>& contactnorm)
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
        
    if( dot3(vnorm, u1) + vnorm.w > 0 ) b |= 1;
    if( dot3(vnorm, u2) + vnorm.w > 0 ) b |= 2;
    if( dot3(vnorm, u3) + vnorm.w > 0 ) b |= 4;
    
    if(b == 7 || b == 0) return false;
    
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
        //const RaveVector<T>* pnext = pv[i+1];
        
        RaveVector<T> q1 = p1 - *pprev;
        RaveVector<T> q2 = p2 - *pprev;
        cross3(vcross, vedges[i], vnorm);

        T t1 = dot3(q1, vcross);
        T t2 = dot3(q2, vcross);

        // line segment is out of face
        if( t1 >= 0 && t2 >= 0 )
            return false;

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

bool TriTriCollision(const RaveVector<float>& u1, const RaveVector<float>& u2, const RaveVector<float>& u3,
                   const RaveVector<float>& v1, const RaveVector<float>& v2, const RaveVector<float>& v3,
                     RaveVector<float>& contactpos, RaveVector<float>& contactnorm)
{
    return TriTriCollision_temp<float>(u1,u2,u3,v1,v2,v3,contactpos,contactnorm);
}

bool TriTriCollision(const RaveVector<double>& u1, const RaveVector<double>& u2, const RaveVector<double>& u3,
                   const RaveVector<double>& v1, const RaveVector<double>& v2, const RaveVector<double>& v3,
                     RaveVector<double>& contactpos, RaveVector<double>& contactnorm)
{
    return TriTriCollision_temp<double>(u1,u2,u3,v1,v2,v3,contactpos,contactnorm);
}

OBB OBBFromAABB(const AABB& ab, const TransformMatrix& t)
{
    OBB o;
    o.right = Vector(t.m[0],t.m[4],t.m[8]);
    o.up = Vector(t.m[1],t.m[5],t.m[9]);
    o.dir = Vector(t.m[2],t.m[6],t.m[10]);
    o.pos = t*ab.pos;
    o.extents = ab.extents;
    return o;
}

OBB TransformOBB(const OBB& obb, const Transform& t)
{
    OBB newobb;
    newobb.extents = obb.extents;
    newobb.pos = t*obb.pos;
    newobb.right = t.rotate(obb.right);
    newobb.up = t.rotate(obb.up);
    newobb.dir = t.rotate(obb.dir);
    return newobb;
}

bool AABBCollision(const AABB& ab1, const AABB& ab2)
{
    Vector v = ab1.pos-ab2.pos;
    return RaveFabs(v.x) <= ab1.extents.x+ab2.extents.x && RaveFabs(v.y) <= ab1.extents.y+ab2.extents.y && RaveFabs(v.z) <= ab1.extents.z+ab2.extents.z;
}

} // end namespace OpenRAVE
