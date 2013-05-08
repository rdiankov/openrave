#include "ParabolicPathSmooth/DynamicPath.h"

namespace ParabolicRamp = ParabolicRampInternal;

bool IterativeMergeRamps(std::list<ParabolicRamp::ParabolicRampND>& origramps,std::list<ParabolicRamp::ParabolicRampND>& ramps, dReal& testcoef, dReal minswitchtime,ConstraintTrajectoryTimingParametersPtr params,dReal maxcoef, dReal precision, int iters);


bool MergeRamps(const ParabolicRamp::ParabolicRampND& ramp0,const ParabolicRamp::ParabolicRampND& ramp1,const ParabolicRamp::ParabolicRampND& ramp2,ParabolicRamp::ParabolicRampND& resramp0,ParabolicRamp::ParabolicRampND& resramp1,ConstraintTrajectoryTimingParametersPtr params);


void BreakIntoUnitaryRamps(const std::list<ParabolicRamp::ParabolicRampND>& ramps,std::list<ParabolicRamp::ParabolicRampND>& resramps,dReal minswitchtime);


void TimeScale(std::list<ParabolicRamp::ParabolicRampND>& origramps,std::list<ParabolicRamp::ParabolicRampND>& ramps,dReal coef);

void TimeScaleOne(ParabolicRamp::ParabolicRampND& ramp,dReal coef);


dReal DetermineMinswitchtime(ParabolicRamp::ParabolicRampND rampnd);
