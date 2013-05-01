#include "ParabolicPathSmooth/DynamicPath.h"

namespace ParabolicRamp = ParabolicRampInternal;


bool MergeWaypoints(const ParabolicRamp::ParabolicRampND& ramp0,const ParabolicRamp::ParabolicRampND& ramp1,const ParabolicRamp::ParabolicRampND& ramp2,ParabolicRamp::ParabolicRampND& resramp0,ParabolicRamp::ParabolicRampND& resramp1,const std::vector<dReal>& qmin,const std::vector<dReal>& qmax,const std::vector<dReal>& vmax,const std::vector<dReal>& amax);

void BreakIntoUnitaryRamps(const std::list<ParabolicRamp::ParabolicRampND>& ramps,std::list<ParabolicRamp::ParabolicRampND>& resramps);

void TimeScale(std::list<ParabolicRamp::ParabolicRampND>& ramps,dReal coef);
