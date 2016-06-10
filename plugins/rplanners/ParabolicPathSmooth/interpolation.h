//-*- coding: utf-8 -*-
#ifndef PARABOLIC_INTERPOLATION_H
#define PARABOLIC_INTERPOLATION_H

#include "ramp.h"
typedef double IkReal;
#define DEG 4
#define IKabs fabs

namespace ParabolicRampInternal {

////////////////////////////////////////////////////////////////////////////////
// Multi DOF
ParabolicCurvesND InterpolateZeroVelND(std::vector<Real>& x0Vect, std::vector<Real>& x1Vect,
                                       std::vector<Real>& vmVect, std::vector<Real>& amVect, Real delta=0);
ParabolicCurvesND InterpolateArbitraryVelND(std::vector<Real>& x0Vect, std::vector<Real>& x1Vect,
                                            std::vector<Real>& v0Vect, std::vector<Real>& v1Vect,
                                            std::vector<Real>& vmVect, std::vector<Real>& amVect, Real delta=0);
ParabolicCurvesND ReinterpolateNDFixedDuration(std::vector<ParabolicCurve> curves,
                                               std::vector<Real> vmVect, std::vector<Real> amVect, int maxIndex, Real delta=0);

////////////////////////////////////////////////////////////////////////////////
// Single DOF
ParabolicCurve Interpolate1D(Real x0, Real x1, Real v0, Real v1, Real vm, Real am, Real delta=0);
ParabolicCurve Interpolate1DNoVelocityLimit(Real x0, Real x1, Real v0, Real v1, Real am);
ParabolicCurve ImposeVelocityLimit(ParabolicCurve curve, Real vm);
ParabolicCurve Stretch1D(ParabolicCurve curve, Real newDuration, Real vm, Real am);

////////////////////////////////////////////////////////////////////////////////
// Utilities
bool _SolveForT0(Real A, Real B, Real t, Real tLow, Real tHigh, Real& t0);

// Compute roots of a quartic polynomial. This functions is taken from OpenRAVE ik generator file.
static inline void FindPolyRoots4(IkReal rawcoeffs[DEG + 1], IkReal rawroots[DEG], int& numroots)
{
    using std::complex;
    if( rawcoeffs[0] == 0 ) {
        // solve with one reduced degree
        //%(reducedpolyroots)s(&rawcoeffs[1], &rawroots[0], numroots);
        numroots = 0;
        return;
    }
    const IkReal tol = 128.0*std::numeric_limits<IkReal>::epsilon();
    const IkReal tolsqrt = sqrt(std::numeric_limits<IkReal>::epsilon());
    complex<IkReal> coeffs[DEG];
    const int maxsteps = 110;
    for(int i = 0; i < DEG; ++i) {
        coeffs[i] = complex<IkReal>(rawcoeffs[i+1]/rawcoeffs[0]);
    }
    complex<IkReal> roots[DEG];
    IkReal err[DEG];
    roots[0] = complex<IkReal>(1,0);
    roots[1] = complex<IkReal>(0.4,0.9); // any complex number not a root of unity works
    err[0] = 1.0;
    err[1] = 1.0;
    for(int i = 2; i < DEG; ++i) {
        roots[i] = roots[i-1]*roots[1];
        err[i] = 1.0;
    }
    for(int step = 0; step < maxsteps; ++step) {
        bool changed = false;
        for(int i = 0; i < DEG; ++i) {
            if ( err[i] >= tol ) {
                changed = true;
                // evaluate
                complex<IkReal> x = roots[i] + coeffs[0];
                for(int j = 1; j < DEG; ++j) {
                    x = roots[i] * x + coeffs[j];
                }
                for(int j = 0; j < DEG; ++j) {
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
    bool visited[DEG] = {false};
    for(int i = 0; i < DEG; ++i) {
        if( !visited[i] ) {
            // might be a multiple root, in which case it will have more error than the other roots
            // find any neighboring roots, and take the average
            complex<IkReal> newroot=roots[i];
            int n = 1;
            for(int j = i+1; j < DEG; ++j) {
                // care about error in real much more than imaginary
                if( std::abs(real(roots[i])-real(roots[j])) < tolsqrt && std::abs(imag(roots[i])-imag(roots[j])) < 0.002 ) {
                    newroot += roots[j];
                    n += 1;
                    visited[j] = true;
                }
            }
            if( n > 1 ) {
                newroot /= n;
            }
            // there are still cases where even the mean is not accurate enough, until a better
            // multi-root algorithm is used, need to use the sqrt
            if( IKabs(imag(newroot)) < tolsqrt ) {
                rawroots[numroots++] = real(newroot);
            }
        }
    }
}


class Interval {
public:
    Interval(Real lIn=-inf, Real hIn=inf) : l(lIn), h(hIn), isVoid(lIn <= hIn) {
    }
    ~Interval() {
    }

    // Functions
    bool Contain(Real x) const {
        if ((l <= x) && (x <= h)) {
            return true;
        }
        else {
            return false;
        }
    }

    bool CheckIntersection(Interval& i0) const {
        if ((i0.l > h) || (i0.h < l)) {
            return false;
        }
        else {
            return true;
        }
    }

    // Modifies its own interval to be the intersection
    void Intersect(Interval& i0) {
        if ((i0.l > h) || (i0.h < l)) {
            l = inf;
            h = -inf;
            isVoid = true;
            return;
        }
        else {
            l = Max(i0.l, l);
            h = Min(i0.h, h);
            isVoid = (l <= h);
        }
    }

    // Members
    Real l, h;
    bool isVoid;
}; // end class Interval

} // namespace ParabolicRampInternal
#endif
