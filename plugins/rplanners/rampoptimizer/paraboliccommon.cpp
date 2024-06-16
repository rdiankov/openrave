/*****************************************************************************
 *
 * Copyright (c) 2010-2011, the Trustees of Indiana University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Indiana University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE TRUSTEES OF INDIANA UNIVERSITY ''AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE TRUSTEES OF INDIANA UNIVERSITY BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***************************************************************************/

#include "paraboliccommon.h"

namespace OpenRAVE {

namespace RampOptimizerInternal {

//a flag used during testing of failed ramps
bool gSuppressSavingRamps = false;

//a flag used to stop SolveMinAccel from complaining during internal testing
bool gMinAccelQuiet = false;

//a flag used to stop SolveMinTime2 from complaining during internal testing
bool gMinTime2Quiet = false;

//return a value x in [xmin,xmax] such that |a*x - b| <= epsilon*max(|a||b|)
//for ill posed problems choose x=0
bool SafeEqSolve(dReal a,dReal b,dReal epsilon,dReal xmin,dReal xmax,dReal& x)
{
    if(a < 0) {
        return SafeEqSolve(-a,-b,epsilon,xmin,xmax,x);
    }

    //now we know a>=0
    dReal epsScaled = epsilon*Max(a,Abs(b));

    //infinte range
    if(IsInf(xmin)==-1 && IsInf(xmax)==1) {
        if(a == 0) {
            x = 0.0;
            return Abs(b) <= epsScaled;
        }
        x = b/a;
        return true;
    }

    dReal axmin=a*xmin,axmax=a*xmax;
    if(!(b + epsScaled >= axmin  && b - epsScaled <= axmax)) { //ranges don't intersect
        return false;
    }
    if(a != 0) {
        x = b/a;
        if(xmin <= x && x <= xmax) {
            return true;
        }
    }
    if (Abs(0.5*(axmin+axmax) - b) <= epsScaled) {
        //center of range is in epsilon range
        x = 0.5*(xmin+xmax);
        return true;
    }
    if(Abs(axmax - b) <= epsScaled) {
        x = xmax;
        return true;
    }
    assert(Abs(axmin - b) <= epsScaled);
    x = xmin;
    return true;
}

} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
