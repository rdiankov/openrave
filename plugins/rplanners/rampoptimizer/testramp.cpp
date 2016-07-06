#include "ramp.h"
#include <iostream>
#include <boost/format.hpp>
#include <assert.h>
using namespace RampOptimizerInternal;

int main() {
    ////////////////////////////////////////////////////////////////////////////////
    // Ramp
    
    if (1) { // limit the scope of variables
        // Simple test: initialization and evaluation
        Ramp ramp0(0.0, 1.1, 2.2, 3.3);
        assert(FuzzyEquals(ramp0.x0, 3.3, epsilon));
        assert(FuzzyEquals(ramp0.d, 2.662, epsilon));

        assert(FuzzyEquals(ramp0.EvalPos(0), 3.3, epsilon));
        assert(FuzzyEquals(ramp0.EvalPos(ramp0.duration), 5.962, epsilon));
        assert(FuzzyEquals(ramp0.EvalPos(1.1), 3.9655, epsilon));

        assert(FuzzyEquals(ramp0.EvalVel(0), 0, epsilon));
        assert(FuzzyEquals(ramp0.EvalVel(ramp0.duration), 2.42, epsilon));
        assert(FuzzyEquals(ramp0.EvalVel(1.1), 1.21, epsilon));

        Ramp ramp1;
        ramp1.Initialize(0.0, 1.1, 2.2, 3.3);
        assert(FuzzyEquals(ramp1.x0, 3.3, epsilon));
        assert(FuzzyEquals(ramp1.d, 2.662, epsilon));

        assert(FuzzyEquals(ramp1.EvalPos(0), 3.3, epsilon));
        assert(FuzzyEquals(ramp1.EvalPos(ramp1.duration), 5.962, epsilon));
        assert(FuzzyEquals(ramp1.EvalPos(1.1), 3.9655, epsilon));

        assert(FuzzyEquals(ramp1.EvalVel(0), 0, epsilon));
        assert(FuzzyEquals(ramp1.EvalVel(ramp1.duration), 2.42, epsilon));
        assert(FuzzyEquals(ramp1.EvalVel(1.1), 1.21, epsilon));

        std::cout << "[Ramp] Test initialization and evaluation: done" << std::endl;

        // Test GetPeaks: normal situation where the peaks are at end points
        Real bmin, bmax;
        ramp0.GetPeaks(bmin, bmax);
        assert(FuzzyEquals(bmin, 3.3, epsilon));
        assert(FuzzyEquals(bmax, 5.962, epsilon));

        // Test GetPeaks: one of the peaks is not an end point
        Ramp ramp2(-0.59, 1.75, 0.897, 1.4);
        ramp2.GetPeaks(bmin, bmax);
        assert(FuzzyEquals(bmin, 1.3005428571428571, epsilon));
        assert(FuzzyEquals(bmax, 1.574802875, epsilon));

        std::cout << "[Ramp] Test GetPeaks: done" << std::endl;
    }
    ////////////////////////////////////////////////////////////////////////////////
    // ParabolicCurve

    if (1) {
        Ramp ramp0(2.8, -1.95, 0.99, 0.04);
        Ramp ramp1(0.8695, 1.93, 1.11);
        std::vector<Ramp> ramps0(2);
        ramps0[0] = ramp0;
        ramps0[1] = ramp1;
        ParabolicCurve curve0(ramps0);

        assert(FuzzyEquals(curve0.x0, 0.04, epsilon));
        assert(FuzzyEquals(curve0.d, 3.970524, epsilon));

        assert(FuzzyEquals(curve0.EvalPos(0), 0.04, epsilon));
        assert(FuzzyEquals(curve0.ramps[0].EvalPos(curve0.ramps[0].duration), curve0.ramps[1].x0, epsilon));
        assert(FuzzyEquals(curve0.EvalPos(curve0.duration), 4.010524, epsilon));

        std::cout << "[ParabolicCurve] Test initialization and evaluation: done" << std::endl;
    }




    
    return 0;
}
