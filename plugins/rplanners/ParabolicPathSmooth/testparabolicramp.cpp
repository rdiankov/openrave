#include "ppramp.h"

using namespace ParabolicRampInternal;
int main()
{
    PPRamp ppramp;
    ppramp.x0 = 6.787962959079534e-01;
    ppramp.dx0=2.083090465764151e-02;
    ppramp.x1=8.617416163754724e-01;
    ppramp.dx1=1.522458717201383e-01;
    bool bsuccess = ppramp.SolveMinAccel(0.008);
    RAVELOG_WARN("success=%d\n", (int)bsuccess);
}
