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
namespace mathextra {

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

void EigenSymmetric3(const double* fmat, double* afEigenvalue, double* fevecs)
{
    double afSubDiag[3];

    memcpy(fevecs, fmat, sizeof(double)*9);
    Tridiagonal3(fevecs, afEigenvalue,afSubDiag);
    QLAlgorithm3(fevecs, afEigenvalue,afSubDiag);

    // make eigenvectors form a right--handed system
    double fDet = fevecs[0*3+0] * (fevecs[1*3+1] * fevecs[2*3+2] - fevecs[1*3+2] * fevecs[2*3+1]) +
                  fevecs[0*3+1] * (fevecs[1*3+2] * fevecs[2*3+0] - fevecs[1*3+0] * fevecs[2*3+2]) +
                  fevecs[0*3+2] * (fevecs[1*3+0] * fevecs[2*3+1] - fevecs[1*3+1] * fevecs[2*3+0]);
    if ( fDet < 0.0f )
    {
        fevecs[0*3+2] = -fevecs[0*3+2];
        fevecs[1*3+2] = -fevecs[1*3+2];
        fevecs[2*3+2] = -fevecs[2*3+2];
    }
}
/* end of MAGIC code */

} // end namespace geometry
} // end namespace OpenRAVE
