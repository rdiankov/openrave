#include <iostream>
#include <boost/format.hpp>
#include "ramp.h"
#include "interpolation.h"
#include "parabolicchecker.h"
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

void ScalarMultVector(Real a, std::vector<Real>& A, std::vector<Real>& R) {
    size_t sz = A.size();
    R.clear();
    R.reserve(sz);
    for (size_t i = 0; i != sz; ++i) {
        R.push_back(a*A[i]);
    }
}

bool CheckRamp(Ramp ramp, Real vm, Real am) {
    bool result = (Abs(ramp.v0) < vm + epsilon) && (Abs(ramp.v1) < vm + epsilon) && (Abs(ramp.a) < am + epsilon);
    return result;
}

bool FuzzyVectorEquals(std::vector<Real>& A, std::vector<Real>& B, Real eps) {
    BOOST_ASSERT(A.size() == B.size());
    bool passed = true;
    for (size_t i = 0; i < A.size(); ++i) {
        passed = (passed && FuzzyEquals(A[i], B[i], eps));
    }
    return passed;
}

int  main() {
    bool passed;
    std::cout << "This is a test file for new Ramp structure" << std::endl;
    std::cout << str(boost::format("Error tolerance = %.15e")%epsilon) << std::endl;
    Ramp ramp1(0, 2, 1, 0);
    Ramp ramp2(ramp1.v1, -ramp1.a, 0.8, 0);
    std::vector<Ramp> ramps;
    ramps.push_back(ramp1);
    ramps.push_back(ramp2);
    ParabolicCurve curve1(ramps);
    // std::cout << str(boost::format("displacement = %.15e")%curve1.d) << std::endl;
    passed = FuzzyEquals(curve1.duration, 1.8, epsilon);
    std::cout << str(boost::format("Checking duration addition (1): %d")%passed) << std::endl;
    passed = FuzzyEquals(curve1.ramps[0].v1, curve1.ramps[1].v0, epsilon);
    std::cout << str(boost::format("Checking velocity continuity (1): %d")%passed) << std::endl;

    ParabolicCurve curve2;
    ramps.clear();
    Ramp ramp3(0.1, 1.7, 0.8, 1);
    Ramp ramp4(ramp3.v1, 0, 0.27);
    Ramp ramp5(ramp4.v1, -1.7, 0.73);
    ramps.push_back(ramp3);
    ramps.push_back(ramp4);
    ramps.push_back(ramp5);
    curve2.Initialize(ramps);
    passed = FuzzyEquals(curve2.duration, 1.8, epsilon);
    std::cout << str(boost::format("Checking duration addition (2): %d")%passed) << std::endl;
    // curve2.PrintInfo();

    std::vector<ParabolicCurve> curves(2);
    curves[0] = curve1;
    curves[1] = curve2;
    ParabolicCurvesND curvesnd;
    curvesnd.Initialize(curves);
    passed = FuzzyEquals(curvesnd.duration, 1.8, epsilon);
    std::cout << str(boost::format("Checking CurvesND duration: %d")%passed) << std::endl;

    // curvesnd.PrintInfo();

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
    std::vector<Real> x0Vect(6), x1Vect(6), v0Vect(6), v1Vect(6);
    x0Vect = RandomVector(xMin, xMax);
    x1Vect = RandomVector(xMin, xMax);
    std::cout << "x0Vect = " << GenerateStringFromVector(x0Vect) << std::endl;
    std::cout << "x1Vect = " << GenerateStringFromVector(x1Vect) << std::endl;

    std::vector<Real> dVect;
    LinComVector(1, x1Vect, -1, x0Vect, dVect);
    std::cout << "dVect  = " << GenerateStringFromVector(dVect) << std::endl;
    ParabolicCurvesND curvesnd2 = InterpolateZeroVelND(x0Vect, x1Vect, vMax, aMax);

    std::vector<Real> discr1, discr2;
    std::cout << str(boost::format("duration = %.15e")%curvesnd2.duration) << std::endl;
    std::vector<Real> newdVect = curvesnd2.dVect;
    std::vector<Real> resDisplacement = curvesnd2.EvalPos(curvesnd2.duration);
    LinComVector(1, dVect, -1, newdVect, discr1);
    std::cout << "discrepancy1 = " << GenerateStringFromVector(discr1) << std::endl;

    // std::cout << GenerateStringFromVector(x0Vect) << std::endl;
    // std::cout << GenerateStringFromVector(curvesnd2.x0Vect) << std::endl;
    // std::cout << GenerateStringFromVector(dVect) << std::endl;
    // std::cout << GenerateStringFromVector(curvesnd2.dVect) << std::endl;

    ////////////////////////////////////////////////////////////////////////////////
    // Problem instaces from testappaskulbox0
    // std::vector<Real> vMaxAskul(6), aMaxAskul(6), vMinAskul(6);
    // vMaxAskul[0] = 1.673790414395085e+00;
    // vMaxAskul[1] = 9.743973935330699e-01;
    // vMaxAskul[2] = 1.169276872239683e+00;
    // vMaxAskul[3] = 1.461596090299605e+00;
    // vMaxAskul[4] = 1.461596090299605e+00;
    // vMaxAskul[5] = 2.338553744479367e+00;
    // aMaxAskul[0] = 5.407537373884282e+00;
    // aMaxAskul[1] = 4.135175638852687e+00;
    // aMaxAskul[2] = 5.681095146916074e+00;
    // aMaxAskul[3] = 1.145125561528436e+01;
    // aMaxAskul[4] = 1.042064260990877e+01;
    // aMaxAskul[5] = 1.405959717209913e+01;
    // ScalarMultVector(-1, vMaxAskul, vMinAskul);
    // std::vector<Real> vMin;
    // ScalarMultVector(-1, vMax, vMin);



    

    int nTrials = 1000;
    int nSuccess = 0;
    int interpfailed = 0;
    int vboundfailed = 0;
    int aboundfailed = 0;
    int negduration = 0;
    int vdiscont = 0;
    int vdisc = 0;
    int xdisc = 0;
    int durdisc = 0;

    ParabolicCheckReturn ret;
    std::vector<Real> temp;
    for (int i = 0; i < nTrials; ++i) {
        x0Vect = RandomVector(xMin, xMax);
        x1Vect = RandomVector(xMin, xMax);
        v0Vect = RandomVector(vMin, vMax);
        v1Vect = RandomVector(vMin, vMax);
        LinComVector(1, x1Vect, -1, x0Vect, dVect);
        bool allPassed = true;

        ParabolicCurvesND curvesnd3 = InterpolateArbitraryVelND(x0Vect, x1Vect, v0Vect, v1Vect, vMax, aMax);
        // std::cout << "INTERPOLATION INSTANCE " << i << " FINISHED" << std::endl;
        // std::cout << std::endl;
        if (curvesnd3.IsEmpty()) {
            // std::cout << "INTERPOLATION INSTANCE FAILED" << std::endl;
            interpfailed += 1;
            continue;
        }

        ret = CheckParabolicCurvesND(curvesnd3, vMax, aMax, v0Vect, v1Vect, x0Vect, x1Vect);
        // std::cout << "RETURN" << ret << std::endl;
        switch(ret) {
        case PCR_Normal: nSuccess += 1; break;
        case PCR_NegativeDuration: negduration += 1; break;
        case PCR_VBoundViolated: vboundfailed += 1; break;
        case PCR_ABoundViolated: aboundfailed += 1; break;
        case PCR_VDiscontinuous: vdiscont += 1; break;
        case PCR_XDiscrepancy: xdisc += 1;  break;
        case PCR_VDiscrepancy: vdisc += 1; break;
        case PCR_DurationDiscrepancy:; durdisc += 1; break;
        }
        /*
           for (int j = 0; j < curvesnd3.ndof; ++j) {
            // Check each DOF
            for (int k = 0; k < (int) curvesnd3.curves[j].ramps.size(); ++k) {
                if (k == 0) {
                    passed = FuzzyEquals(curvesnd3.curves[j].ramps[k].x0, x0Vect[j], epsilon);
                    if (!passed) {
                        std::cout << str(boost::format("DOF %d; ramp %d: initial displacement failed")%j %k) << std::endl;
                        allPassed = false;
                        break;
                    }
                }
                else {
                    passed = FuzzyEquals(curvesnd3.curves[j].ramps[k].v0, curvesnd3.curves[j].ramps[k - 1].v1, epsilon);
                    if (!passed) {
                        std::cout << str(boost::format("DOF %d; ramp %d: initial velocity failed")%j %k) << std::endl;
                        allPassed = false;
                        break;
                    }
                }
                passed = CheckRamp(curvesnd3.curves[j].ramps[k], vMax[j], aMax[j]);
                if (!passed) {
                    std::cout << str(boost::format("DOF %d; ramp %d: CheckRamp failed")%j %k) << std::endl;
                    allPassed = false;
                    vboundfailed += 1;
                    break;
                }
            }
            // Check Curve
            passed = FuzzyEquals(curvesnd3.curves[j].v0, v0Vect[j], epsilon);
            if (!passed) {
                std::cout << str(boost::format("DOF %d; Curve initial velocity failed")%j) << std::endl;
                allPassed = false;
                break;
            }
            passed = FuzzyEquals(curvesnd3.curves[j].v1, v1Vect[j], epsilon);
            if (!passed) {
                std::cout << str(boost::format("DOF %d; Curve final velocity failed")%j) << std::endl;
                std::cout << curvesnd3.v1Vect[j] << " " << v1Vect[j] << std::endl;
                allPassed = false;
                break;
            }
            passed = FuzzyEquals(curvesnd3.curves[j].d, dVect[j], epsilon);
            if (!passed) {
                std::cout << str(boost::format("DOF %d; Curve displacement failed")%j) << std::endl;
                allPassed = false;
                break;
            }
            passed = FuzzyEquals(curvesnd3.curves[j].x0, x0Vect[j], epsilon);
            if (!passed) {
                std::cout << str(boost::format("DOF %d; Curve initial displacement failed")%j) << std::endl;
                allPassed = false;
                break;
            }
            passed = FuzzyEquals(curvesnd3.curves[j].duration, curvesnd3.duration, epsilon);
            if (!passed) {
                std::cout << str(boost::format("DOF %d; Curve duration failed")%j) << std::endl;
                allPassed = false;
                break;
            }
            // Check CurvesND
            passed = FuzzyEquals(curvesnd3.v0Vect[j], v0Vect[j], epsilon);
            if (!passed) {
                std::cout << str(boost::format("DOF %d; CurvesND initial velocity failed")%j) << std::endl;
                allPassed = false;
                break;
            }
            passed = FuzzyEquals(curvesnd3.v1Vect[j], v1Vect[j], epsilon);
            if (!passed) {
                std::cout << str(boost::format("DOF %d; CurvesND final velocity failed")%j) << std::endl;
                std::cout << curvesnd3.v1Vect[j] << " " << v1Vect[j] << std::endl;
                allPassed = false;
                break;
            }
            passed = FuzzyEquals(curvesnd3.dVect[j], dVect[j], epsilon);
            if (!passed) {
                std::cout << str(boost::format("DOF %d; CurvesND displacement failed")%j) << std::endl;
                allPassed = false;
                break;
            }
            passed = FuzzyEquals(curvesnd3.x0Vect[j], x0Vect[j], epsilon);
            if (!passed) {
                std::cout << str(boost::format("DOF %d; CurvesND initial displacement failed")%j) << std::endl;
                allPassed = false;
                break;
            }
           }
           std::cout << FuzzyEquals(curvesnd3.curves[0].duration, curvesnd3.duration, epsilon) << std::endl;

           temp = curvesnd3.EvalPos(curvesnd3.duration);
           passed = FuzzyVectorEquals(temp, x1Vect, epsilon);
           std::cout << "TEMP = " << GenerateStringFromVector(temp) << std::endl;
           std::cout << "x1Vect = " << GenerateStringFromVector(x1Vect) << std::endl;
           if (!passed) {
            std::cout << str(boost::format("CurvesND final configuration evaluation failed")) << std::endl;
            allPassed = false;
            break;
           }
           if (allPassed) {
            nSuccess += 1;
           }
           else {
            break;
           }

         */

    }



    // std::cout << str(boost::format("running tests with epsilon = %f")%epsilon) << std::endl;
    std::cout << epsilon << std::endl;
    std::cout << str(boost::format("successful instances %d/%d")%nSuccess %nTrials) << std::endl;
    std::cout << str(boost::format("interpolation failed %d/%d")%interpfailed %nTrials) << std::endl;
    std::cout << str(boost::format("v-bound failed %d/%d")%vboundfailed %nTrials) << std::endl;
    std::cout << str(boost::format("a-bound failed %d/%d")%aboundfailed %nTrials) << std::endl;

    // Problem instance taken from python implementation
    x0Vect[0] = 1.933026035708321e+00;
    x0Vect[1] = -9.537655453434970e-01;
    x0Vect[2] = 2.372374655655442e+00;
    x0Vect[3] = 7.173622533524755e-01;
    x0Vect[4] = -4.933040334499035e-01;
    x0Vect[5] = -5.000011212089064e+00;

    x1Vect[0] = 2.822876510392579e+00;
    x1Vect[1] = -1.081375399195004e+00;
    x1Vect[2] = 2.681916512606008e+00;
    x1Vect[3] = -3.594860272272365e+00;
    x1Vect[4] = 3.104360332743843e-01;
    x1Vect[5] = 9.078000785888474e-02;

    v0Vect[0] = 3.224198786400277e+00;
    v0Vect[1] = 1.955868545907482e+00;
    v0Vect[2] = 2.408282596027826e+00;
    v0Vect[3] = 1.567905033961802e+00;
    v0Vect[4] = 1.735766557861855e-01;
    v0Vect[5] = 3.796327365066725e+00;

    v1Vect[0] = 3.096177616566498e+00;
    v1Vect[1] = -6.851652133365980e-01;
    v1Vect[2] = 2.594000046066456e+00;
    v1Vect[3] = 1.961011639149750e-01;
    v1Vect[4] = -2.409444090679572e+00;
    v1Vect[5] = 1.345540921049356e+00;

    vMax[0] = 3.926990816987241e+00;
    vMax[1] = 2.617993877991494e+00;
    vMax[2] = 2.858325715991114e+00;
    vMax[3] = 3.926990816987241e+00;
    vMax[4] = 3.021688533977783e+00;
    vMax[5] = 6.283185307179586e+00;

    aMax[0] = 9.866782593836945e+00;
    aMax[1] = 8.422348104886435e+00;
    aMax[2] = 1.035442758684416e+01;
    aMax[3] = 1.048323288564134e+01;
    aMax[4] = 1.186143212947866e+01;
    aMax[5] = 1.675516081914556e+01;

    ParabolicCurvesND curvesndtest = InterpolateArbitraryVelND(x0Vect, x1Vect, v0Vect, v1Vect, vMax, aMax);
    int checkret = CheckParabolicCurvesND(curvesndtest, vMax, aMax, v0Vect, v1Vect, x0Vect, x1Vect);
    std::cout << "RESULT = " << checkret << std::endl;
}
