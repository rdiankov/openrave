#include "ramp.h"
#include "interpolation.h"
#include <iostream>
#include <openrave/openrave.h>

using namespace RampOptimizerInternal;

int main() {
    bool res;

    OpenRAVE::RaveSetDebugLevel(OpenRAVE::Level_Verbose);
    
    ////////////////////////////////////////////////////////////////////////////////
    // Single DOF
    if (1) { // Test Interpolate1D
        // Zero terminal velocities
        ParabolicCurve curve0;
        res = Interpolate1D(0, 1.55, 0, 0, 100, 0.67, curve0);
        assert(res);
        assert(FuzzyEquals(curve0.ramps[0].v1, 1.019068201839307, epsilon));
        assert(FuzzyEquals(curve0.duration, 3.04199463235614, epsilon));
        assert(FuzzyEquals(curve0.ramps[0].a, 0.67, epsilon));
        assert(FuzzyEquals(curve0.ramps[1].a, -0.67, epsilon));
        std::cout << "[Interpolation1D] zero terminal velocities + two-ramp: done" << std::endl;
        
        ParabolicCurve curve1;
        res = Interpolate1D(0, -1.55, 0, 0, 0.58, 0.67, curve1);
        assert(res);
        assert(FuzzyEquals(curve1.ramps[0].a, -0.67, epsilon));
        assert(FuzzyEquals(curve1.ramps[2].a, 0.67, epsilon));
        assert(FuzzyEquals(curve1.ramps[1].v0, -0.58, epsilon));
        assert(FuzzyEquals(curve1.ramps[1].a, 0, epsilon));
        assert(FuzzyEquals(curve1.duration, 3.538085434894493, epsilon));
        assert(FuzzyEquals(curve1.ramps[0].duration, 0.865671641791044, epsilon));
        assert(FuzzyEquals(curve1.ramps[2].duration, 0.865671641791044, epsilon));
        assert(FuzzyEquals(curve1.ramps[1].duration, 1.806742151312403, epsilon));
        std::cout << "[Interpolation1D] zero terminal velocities + three-ramp: done" << std::endl;

        ParabolicCurve curve2;
        res = Interpolate1D(1.7610371368798483, -2.2212539670710796, 1.7482937617689935, 0.49265119448402972, 100, 9.8667825938369447, curve2);
        assert(res);
        assert(FuzzyEquals(curve2.ramps[0].v1, -6.398595055673408, epsilon));
        assert(FuzzyEquals(curve2.duration, 1.524117403477913, epsilon));
        assert(FuzzyEquals(curve2.ramps[0].a, -9.8667825938369447, epsilon));
        assert(FuzzyEquals(curve2.ramps[1].a, 9.8667825938369447, epsilon));
        std::cout << "[Interpolation1D] arbitrary terminal velocities + two-ramp: done" << std::endl;
        
        ParabolicCurve curve3;
        res = Interpolate1D(1.7610371368798483, -2.2212539670710796, 1.7482937617689935, 0.49265119448402972, 3.926990816987241, 9.8667825938369447, curve3);
        assert(res);
        assert(FuzzyEquals(curve3.ramps[0].a, -9.8667825938369447, epsilon));
        assert(FuzzyEquals(curve3.ramps[2].a, 9.8667825938369447, epsilon));
        assert(FuzzyEquals(curve3.ramps[1].v0, -3.926990816987241, epsilon));
        assert(FuzzyEquals(curve3.ramps[1].a, 0, epsilon));
        assert(FuzzyEquals(curve3.duration, 1.681777728657942, epsilon));
        assert(FuzzyEquals(curve3.ramps[0].duration, 0.5751910032254251, epsilon));
        assert(FuzzyEquals(curve3.ramps[2].duration, 0.4479314274373387, epsilon));
        assert(FuzzyEquals(curve3.ramps[1].duration, 0.6586552979951781, epsilon));
        std::cout << "[Interpolation1D] arbitrary terminal velocities + three-ramp: done" << std::endl;
    }

    if (1) { // Test Stretch1D
        // The computed profile has two ramps (vp does not exceed vm)
        ParabolicCurve curve0, curve1;
        res = Interpolate1D(1.334850288597095, 1.3345299669274491, 0, 0, 4.3943027242087229, 24.033183799961918, curve0);
        assert(res);
        res = Stretch1D(curve0, 0.11880652979425915, 4.3943027242087229, 24.033183799961918, curve1);
        assert(res);
        assert(FuzzyEquals(curve1.duration, 0.11880652979425915, epsilon));
        assert(FuzzyEquals(curve1.ramps[0].a, -0.09077488120406535, epsilon));
        assert(FuzzyEquals(curve1.ramps[1].a, 0.09077488120406535, epsilon));
        std::cout << "[Stretch1D] case: the stretched profile has two ramps (vp does not exceed vm): done" << std::endl;
        
        // The computed profile has one ramp (t0 is either 0 or newDuration)
        ParabolicCurve curve2, curve3;
        res = Interpolate1D(0.16383770895191763, 0.15318984750484704, -0.81134691912168311, -0.79944445258593466, 1.4940718558591721, 7.5136307267887785, curve2);
        assert(res);
        res = Stretch1D(curve2, 0.01322065865771640, 1.4940718558591721, 7.5136307267887785, curve3);
        assert(curve3.ramps.size() == 1); // caution: this may raise assertion failure if epsilon is not 1e-10 (eg., smaller than that)
        assert(FuzzyEquals(curve3.ramps[0].a, 0.9002930068693192, epsilon));
        std::cout << "[Stretch1D] case: the stretched profile has one ramp: done" << std::endl;

        // Interval 2 and 4 intersect at only one point:: the given newDuratio is the trajectory's minimum time
        ParabolicCurve curve4, curve5;
        res = Interpolate1D(1.032791860941638, 1.0597145622491539, 1.0655552189873183, 0.042986108496728709, 3.8975383858598369, 18.378317023500291, curve4);
        assert(res);
        res = Stretch1D(curve4, 0.08988795330297148, 3.8975383858598369, 18.378317023500291, curve5);
        assert(res);
        assert(FuzzyEquals(curve5.duration, 0.08988795330297148, epsilon));
        assert(FuzzyEquals(curve5.ramps[0].a, -18.378317023500291, epsilon));
        assert(FuzzyEquals(curve5.ramps[1].a, 18.378317023500291, epsilon));
        assert(FuzzyEquals(curve5.ramps[0].duration, 0.07276396444424809, epsilon));
        assert(FuzzyEquals(curve5.ramps[1].duration, 0.01712398885872338, epsilon));
        std::cout << "[Stretch1D] case: the given newDuration is the trajectory's minimum time: done" << std::endl;

        // The new a0 is zero
        ParabolicCurve curve6, curve7;
        res = Interpolate1D(0.82531750296556994, 0.79009174178799435, -0.19471127141398498, -0.086230085825425309, 0.19471127141398498, 9.4898798419620984, curve6);
        assert(res);
        res = Stretch1D(curve6, 0.18754320583325224, 0.19471127141398498, 9.4898798419620984, curve7);
        assert(res);
        assert(FuzzyEquals(curve7.duration, 0.18754320583325224, epsilon));
        assert(curve7.ramps[0].a == 0);
        assert(FuzzyEquals(curve7.ramps[1].a, 4.557719609629845, epsilon));
        std::cout << "[Stretch1D] case: the new a0 is zero: done" << std::endl;

        // The new a1 is zero
        ParabolicCurve curve8, curve9;
        res = Interpolate1D(1.8135831340945454, 1.6863191822455341, 0.034600795245669787, -0.72568556018283259, 0.72568556018283259, 6.9421294516922218, curve8);
        assert(res);
        res = Stretch1D(curve8, 0.24859854263193015, 0.72568556018283259, 6.9421294516922218, curve9);
        assert(res);
        assert(FuzzyEquals(curve9.duration, 0.24859854263193015, epsilon));
        assert(curve9.ramps[1].a == 0);
        assert(FuzzyEquals(curve9.ramps[0].a, -5.438753902539083, epsilon));
        std::cout << "[Stretch1D] case: the new a1 is zero: done" << std::endl;

        // When t0 + tLastRamp > newDuration
        ParabolicCurve curve10, curve11;
        res = Interpolate1D(1.3357629018636366, 0.44226482173150211, -0.44939431909168071, 9.3540921260636072, 9.3540921260636072, 62.486277879901003, curve10);
        assert(res);
        res = Stretch1D(curve10, 0.48901336113233929, 9.3540921260636072, 62.486277879901003, curve11);
        assert(res);
        assert(FuzzyEquals(curve11.duration, 0.48901336113233929, epsilon));
        assert(curve11.ramps.size() == 2);
        assert(FuzzyEquals(curve11.ramps[0].v1, -9.3540921260636072, epsilon));
        assert(FuzzyEquals(curve11.ramps[0].a, -48.85129928647601, epsilon));
        assert(FuzzyEquals(curve11.ramps[1].a, 60.99202179016471, epsilon));
        std::cout << "[Stretch1D] case: two-ramp with the peak velocity at the bound" << std::endl;

        // Three-ramp: when the middle ramp is too short
        ParabolicCurve curve12, curve13;
        
        
        // Three-ramp: normal case
        ParabolicCurve curve14, curve15;
        res = Interpolate1D(-0.35192871321859048, 1.334850288597095, 0, 0, 4.3943027242087229, 24.033183799961918, curve14);
        assert(res);
        res = Stretch1D(curve14, 0.568, 4.3943027242087229, 24.033183799961918, curve15);
        assert(res);
        assert(curve15.ramps.size() == 3);
        assert(FuzzyEquals(curve15.duration, 0.568, epsilon));
        std::cout << "[Stretch1D] case: three-ramp, normal" << std::endl;
            
    }
    
    if (1) { // Test ImposeJointLimitFixedDuration
        // Case IIa


        // Case IIb


        // Case III

    }

    ////////////////////////////////////////////////////////////////////////////////
    // Multi DOF
    if (1) { // Test InterpolateZeroVelND

    }

    if (1) { // Test InterpolateArbitraryVelND

    }
    
}
