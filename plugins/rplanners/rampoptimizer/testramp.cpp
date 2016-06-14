#include <iostream>
#include <boost/format.hpp>
#include "ramp.h"
#include "interpolation.h"
#include <openrave/openrave.h>

#include <cstdlib>
#include <stdlib.h>
#include <time.h>

using namespace RampOptimizerInternal;

Real RandomDouble(Real l, Real h) {
    assert(l <= h);
    return (((Real) rand())/ RAND_MAX)*(h - l) + l;
}

std::vector<Real> RandomVector(std::vector<Real>& lVect, std::vector<Real>& hVect) {
    assert(lVect.size() == hVect.size());
    std::vector<Real> r(lVect.size());
    for (size_t i = 0; i != lVect.size(); ++i) {
        r[i] = RandomDouble(lVect[i], hVect[i]);
    }
    return r;
}

void LinComVector(Real a, std::vector<Real>& A, Real b, std::vector<Real>& B, std::vector<Real>& R) {
    size_t sz = A.size();
    assert(sz == B.size());
    R.clear();
    R.reserve(sz);
    for (size_t i = 0; i != sz; ++i) {
        R.push_back((a*A[i]) + (b*B[i]));
    }
}

int main() {
    std::cout << "This is a test file for new Ramp structure" << std::endl;
    std::cout << str(boost::format("Error tolerance = %.15e")%epsilon) << std::endl;
    Ramp ramp1(0, 2, 1, 0);
    Ramp ramp2(ramp1.v1, -ramp1.a, 0.8, 0);
    std::vector<Ramp> ramps;
    ramps.push_back(ramp1);
    ramps.push_back(ramp2);
    ParabolicCurve curve1(ramps);
    std::cout << str(boost::format("displacement = %.15e")%curve1.d) << std::endl;
    std::cout << str(boost::format("duration = %.15e")%curve1.duration) << " (expected duration = 1.8)" << std::endl;
    std::cout << "check duration: " << (curve1.duration == 1.8) << std::endl;
    std::cout << "check velocity continuity: " << curve1.ramps[0].v1 << " " << curve1.ramps[1].v0 << std::endl;

    ParabolicCurve curve2;
    ramps.clear();
    Ramp ramp3(0.1, 1.7, 0.8, 1);
    Ramp ramp4(ramp3.v1, 0, 0.27);
    Ramp ramp5(ramp4.v1, -1.7, 0.73);
    ramps.push_back(ramp3);
    ramps.push_back(ramp4);
    ramps.push_back(ramp5);
    curve2.Initialize(ramps);
    curve2.PrintInfo();
    std::cout << "check duration: " << (curve2.duration == 1.8) << std::endl;

    std::vector<ParabolicCurve> curves(2);
    curves[0] = curve1;
    curves[1] = curve2;
    ParabolicCurvesND curvesnd;
    curvesnd.Initialize(curves);
    curvesnd.PrintInfo();

    ////////////////////////////////////////////////////////////////////////////////
    srand(time(0));
    // Denso VS-060
    std::vector<Real> xMin(6), xMax(6), vMin(6), vMax(6), realAcc(6), aMin(6), aMax(6);
    xMin[0] = -2.96705972839036;
    xMin[1] = -2.094395102393195;
    xMin[2] = -2.181661564992912;
    xMin[3] = -4.71238898038469;
    xMin[4] = -2.094395102393195;
    xMin[5] = -6.283185307179586;
    xMax[0] = 2.96705972839036;
    xMax[1] = 2.094395102393195;
    xMax[2] = 2.705260340591211;
    xMax[3] = 4.71238898038469;
    xMax[4] = 2.094395102393195;
    xMax[5] = 6.283185307179586;

    vMax[0] = 3.926990816987241;
    vMax[1] = 2.617993877991494;
    vMax[2] = 2.858325715991114;
    vMax[3] = 3.926990816987241;
    vMax[4] = 3.021688533977783;
    vMax[5] = 6.283185307179586;

    realAcc[0] = 19.73356518767389;
    realAcc[1] = 16.84469620977287;
    realAcc[2] = 20.70885517368832;
    realAcc[3] = 20.96646577128268;
    realAcc[4] = 23.72286425895733;
    realAcc[5] = 33.51032163829112;

    Real aScale = 0.5;
    for (int i = 0; i < 6; ++i) {
        vMin[i] = -vMax[i];
        aMax[i] = aScale * realAcc[i];
        aMin[i] = -aMax[i];
    }
    ////////////////////////////////////////////////////////////////////////////////
    // Test interpolation routines
    std::vector<Real> x0Vect = RandomVector(xMin, xMax);
    std::vector<Real> x1Vect = RandomVector(xMin, xMax);
    std::cout << GenerateStringFromVector(x0Vect) << std::endl;
    std::cout << GenerateStringFromVector(x1Vect) << std::endl;

    std::vector<Real> dVect;
    LinComVector(1, x1Vect, -1, x0Vect, dVect);
    std::cout << GenerateStringFromVector(dVect) << std::endl;
    ParabolicCurvesND curvesnd2 = InterpolateZeroVelND(x0Vect, x1Vect, vMax, aMax);
    curvesnd2.PrintInfo();
    for (int i = 0; i < 6; ++i) {
        curvesnd2.curves[i].PrintInfo(str(boost::format("DOF %d")%i));
    }

    std::vector<Real> discr;
    std::cout << str(boost::format("duration = %.15e")%curvesnd2.duration) << std::endl;
    std::vector<Real> resDisplacement = curvesnd2.EvalPos(curvesnd2.duration);
    LinComVector(1, dVect, -1, resDisplacement, discr);
    std::cout << "discrepancy = " << GenerateStringFromVector(discr) << std::endl;

    ////////////////////////////////////////////////////////////////////////////////
    // Problem instaces from testappaskulbox0
    std::vector<Real> vMaxAskul(6), aMaxAskul(6);
    vMaxAskul[0] = 1.673790414395085e+00;
    vMaxAskul[1] = 9.743973935330699e-01;
    vMaxAskul[2] = 1.169276872239683e+00;
    vMaxAskul[3] = 1.461596090299605e+00;
    vMaxAskul[4] = 1.461596090299605e+00;
    vMaxAskul[5] = 2.338553744479367e+00;
    aMaxAskul[0] = 5.407537373884282e+00;
    aMaxAskul[1] = 4.135175638852687e+00;
    aMaxAskul[2] = 5.681095146916074e+00;
    aMaxAskul[3] = 1.145125561528436e+01;
    aMaxAskul[4] = 1.042064260990877e+01;
    aMaxAskul[5] = 1.405959717209913e+01;

    std::cout << "vmax_askul = " << GenerateStringFromVector(vMaxAskul) << std::endl;
    std::cout << "amax_askul = " << GenerateStringFromVector(aMaxAskul) << std::endl;
}
