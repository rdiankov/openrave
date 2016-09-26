#include "../ramp.h"
#include "../interpolator.h"
#include "../../ParabolicPathSmooth/ParabolicRamp.h"
#include <iostream>
#include <openrave/openrave.h>
#include "openraveplugindefs.h"
#include <openrave/planningutils.h>
using namespace OpenRAVE;
namespace ParabolicRamp = ParabolicRampInternal;
namespace RampOptimizer = RampOptimizerInternal;

RampOptimizer::ParabolicInterpolator interpolator(6);
// x0Vect
// array([ 0.2070398147,  0.6643523398,  0.8311317761,  4.0889581068,  0.6778553315, -1.4628242847])

// x1Vect
// array([ 2.2405692672, -1.8456896833, -1.2704986774, -3.4531186727, -0.3748698405,  2.7134087835])

// v0Vect
// array([ 1.9868239391,  2.1226284736, -1.9801705882, -2.1660766206,  2.6236451288,  1.2134184021])

// v1Vect
// array([-3.6375057844,  0.8794915285, -1.1205875844,  2.421300871 ,  1.0662902207, -5.2644196043])

void TestCompute1DTrajectory(dReal x0, dReal x1, dReal v0, dReal v1, dReal vm, dReal am, int maxiter)
{
    uint32_t tstart1, tstart2, tend1, tend2;
    dReal totaltime1 = 0, totaltime2 = 0;
    bool res1, res2;

    for (int i = 0; i < maxiter; ++i) {
        ParabolicRamp::ParabolicRamp1D ramp;
        ramp.x0 = x0;
        ramp.x1 = x1;
        ramp.dx0 = v0;
        ramp.dx1 = v1;
        tstart1 = utils::GetNanoTime();
        res1 = ramp.SolveMinTime(am, vm);
        tend1 = utils::GetNanoTime();
        totaltime1 += 0.000000001f*(float)(tend1 - tstart1);

        RampOptimizer::ParabolicCurve curve;
        tstart2 = utils::GetNanoTime();
        res2 = interpolator.Compute1DTrajectory(x0, x1, v0, v1, vm, am, curve);
        tend2 = utils::GetNanoTime();
        totaltime2 += 0.000000001f*(float)(tend2 - tstart2);
    }

    RAVELOG_DEBUG_FORMAT("[1D traj] ParabolicRamp library took %.15e s. for %d interpolations = %.15e s./interpolation", totaltime1%maxiter%(totaltime1/maxiter));
    RAVELOG_DEBUG_FORMAT("[1D traj] RampOptimizer library took %.15e s. for %d interpolations = %.15e s./interpolation", totaltime2%maxiter%(totaltime2/maxiter));
}

void TestCompute1DTrajectoryFixedDuration(dReal x0, dReal x1, dReal v0, dReal v1, dReal vm, dReal am, dReal duration, int maxiter)
{
    uint32_t tstart1, tstart2, tend1, tend2;
    dReal totaltime1 = 0, totaltime2 = 0;
    bool res1, res2;

    // 1D trajectory, no duration given
    x0 = 0.805318442615709;
    x1 = -1.33943968627494;
    v0 = 2.19884434174935;
    v1 = 1.06445962969977;
    vm = 3.02168853397778;
    am = 11.8614321294787;

    for (int i = 0; i < maxiter; ++i) {
        ParabolicRamp::ParabolicRamp1D ramp;
        ramp.x0 = x0;
        ramp.x1 = x1;
        ramp.dx0 = v0;
        ramp.dx1 = v1;
        tstart1 = utils::GetNanoTime();
        res1 = ramp.SolveFixedTime(am, vm, duration);
        tend1 = utils::GetNanoTime();
        totaltime1 += 0.000000001f*(float)(tend1 - tstart1);

        RampOptimizer::ParabolicCurve curve;
        tstart2 = utils::GetNanoTime();
        res2 = interpolator.Compute1DTrajectoryFixedDuration(x0, x1, v0, v1, vm, am, duration, curve);
        tend2 = utils::GetNanoTime();
        totaltime2 += 0.000000001f*(float)(tend2 - tstart2);
    }

    RAVELOG_DEBUG_FORMAT("[1D traj with fixed duration] ParabolicRamp library took %.15e s. for %d interpolations = %.15e s./interpolation", totaltime1%maxiter%(totaltime1/maxiter));
    RAVELOG_DEBUG_FORMAT("[1D traj with fixed duration] RampOptimizer library took %.15e s. for %d interpolations = %.15e s./interpolation", totaltime2%maxiter%(totaltime2/maxiter));
}

void TestComputeZeroVelNDTraj(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect, const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, int maxiter)
{
    uint32_t tstart1, tstart2, tend1, tend2;
    dReal totaltime1 = 0, totaltime2 = 0;
    bool res1, res2;

    std::vector<dReal> vzero(6);
    std::fill(vzero.begin(), vzero.end(), 0);

    for (int i = 0; i < maxiter; ++i) {
        ParabolicRamp::ParabolicRampND ramp;
        ramp.x0 = x0Vect;
        ramp.x1 = x1Vect;
        ramp.dx0 = vzero;
        ramp.dx1 = vzero;
        tstart1 = utils::GetNanoTime();
        res1 = ramp.SolveMinTimeLinear(amVect, vmVect);
        tend1 = utils::GetNanoTime();
        totaltime1 += 0.000000001f*(float)(tend1 - tstart1);

        std::vector<RampOptimizer::RampND> rampndVect;
        tstart2 = utils::GetNanoTime();
        res2 = interpolator.ComputeZeroVelNDTrajectory(x0Vect, x1Vect, vmVect, amVect, rampndVect);
        tend2 = utils::GetNanoTime();
        totaltime2 += 0.000000001f*(float)(tend2 - tstart2);
    }

    RAVELOG_DEBUG_FORMAT("[ND traj, zero vel] ParabolicRamp library took %.15e s. for %d interpolations = %.15e s./interpolation", totaltime1%maxiter%(totaltime1/maxiter));
    RAVELOG_DEBUG_FORMAT("[ND traj, zero vel] RampOptimizer library took %.15e s. for %d interpolations = %.15e s./interpolation", totaltime2%maxiter%(totaltime2/maxiter));
}

void TestComputeArbitraryVelNDTraj(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect, const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect, const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect, const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, int maxiter)
{
    uint32_t tstart1, tstart2, tend1, tend2;
    dReal totaltime1 = 0, totaltime2 = 0;
    bool res1, res2;

    for (int i = 0; i < maxiter; ++i) {
        std::vector<std::vector<ParabolicRamp::ParabolicRamp1D> > ramps;
        tstart1 = utils::GetNanoTime();
        res1 = ParabolicRamp::SolveMinTimeBounded(x0Vect, v0Vect, x1Vect, v1Vect, amVect, vmVect, xminVect, xmaxVect, ramps, 0);
        tend1 = utils::GetNanoTime();
        totaltime1 += 0.000000001f*(float)(tend1 - tstart1);

        std::vector<RampOptimizer::RampND> rampndVect;
        tstart2 = utils::GetNanoTime();
        res2 = interpolator.ComputeArbitraryVelNDTrajectory(x0Vect, x1Vect, v0Vect, v1Vect, xminVect, xmaxVect, vmVect, amVect, rampndVect, true);
        tend2 = utils::GetNanoTime();
        totaltime2 += 0.000000001f*(float)(tend2 - tstart2);
    }

    RAVELOG_DEBUG_FORMAT("[ND traj, arbitrary vel] ParabolicRamp library took %.15e s. for %d interpolations = %.15e s./interpolation", totaltime1%maxiter%(totaltime1/maxiter));
    RAVELOG_DEBUG_FORMAT("[ND traj, arbitrary vel] RampOptimizer library took %.15e s. for %d interpolations = %.15e s./interpolation", totaltime2%maxiter%(totaltime2/maxiter));
}


int main() {
    std::cout << "testinterpolator" << std::endl;
    RaveSetDebugLevel(Level_Debug);
    int maxiter = 10000000;
    dReal x0, x1, v0, v1, vm, am, duration;
    x0 = 0.805318442615709;
    x1 = -1.33943968627494;
    v0 = 2.19884434174935;
    v1 = 1.06445962969977;
    vm = 3.02168853397778;
    am = 11.8614321294787;
    duration = 1.33614018915043;

    TestCompute1DTrajectory(x0, x1, v0, v1, vm, am, maxiter);
    TestCompute1DTrajectoryFixedDuration(x0, x1, v0, v1, vm, am, duration, maxiter);

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // ND trajectory

    std::vector<dReal> x0Vect(6), x1Vect(6), v0Vect(6), v1Vect(6), xminVect(6), xmaxVect(6), vmVect(6), amVect(6);

    xminVect[0] = -2.9670597283903604;
    xminVect[1] = 0;
    xminVect[2] = 0;
    xminVect[3] = -3.1415926535897931;
    xminVect[4] = -2.0943951023931953;
    xminVect[5] = -3.1415926535897931;

    xmaxVect[0] = 2.9670597283903604;
    xmaxVect[1] = 2.3561944901923448;
    xmaxVect[2] = 2.6703537555513241;
    xmaxVect[3] = 3.1415926535897931;
    xmaxVect[4] = 2.0943951023931953;
    xmaxVect[5] = 3.1415926535897931;

    vmVect[0] = 3.9548724517878506;
    vmVect[1] = 3.5077845472738538;
    vmVect[2] = 4.2093414567286223;
    vmVect[3] = 5.2616768209107807;
    vmVect[4] = 5.2616768209107807;
    vmVect[5] = 8.4186829134572463;

    amVect[0] = 19.4668788779691546;
    amVect[1] = 14.8864367890352369;
    amVect[2] = 20.4516739270899492;
    amVect[3] = 41.2239788004052699;
    amVect[4] = 37.5138207083687973;
    amVect[5] = 50.6138850827198183;

    x0Vect[0] = 1.64607201928;
    x0Vect[1] = 1.88442230938;
    x0Vect[2] = 1.01809198505;
    x0Vect[3] = 1.46340380299;
    x0Vect[4] = 0.292319262455;
    x0Vect[5] = 1.10231661669;

    x1Vect[0] = 0.66144929679;
    x1Vect[1] = 2.18064532187;
    x1Vect[2] = 0.179262369807;
    x1Vect[3] = -0.425576900917;
    x1Vect[4] = 0.704935733661;
    x1Vect[5] = -2.77686309323;

    v0Vect[0] = 3.90229671955;
    v0Vect[1] = 1.1775945243;
    v0Vect[2] = 0.156836014857;
    v0Vect[3] = -4.87624362002;
    v0Vect[4] = 3.69194568526;
    v0Vect[5] = 4.16315342579;

    v1Vect[0] = -0.918946952959;
    v1Vect[1] = -1.97502811465;
    v1Vect[2] = 2.8803251112;
    v1Vect[3] = 4.3992599841;
    v1Vect[4] = 2.91801526215;
    v1Vect[5] = -3.05584072714;


    TestComputeZeroVelNDTraj(x0Vect, x1Vect, vmVect, amVect, maxiter);
    TestComputeArbitraryVelNDTraj(x0Vect, x1Vect, v0Vect, v1Vect, xminVect, xmaxVect, vmVect, amVect, maxiter);

    return 0;
}


/*

def PrintHelper(vect, name):
    s = ""
    for i in xrange(len(vect)):
        s += "{0}[{1}] = {2};".format(name, i, vect[i])
        s += "\n"
    return s
    
*/
