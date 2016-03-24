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

namespace ParabolicRampInternal {

//a flag used during testing of failed ramps
bool gSuppressSavingRamps = false;

//a flag used to stop SolveMinAccel from complaining during internal testing
bool gMinAccelQuiet = false;

//a flag used to stop SolveMinTime2 from complaining during internal testing
bool gMinTime2Quiet = false;

//return a value x in [xmin,xmax] such that |a*x - b| <= epsilon*max(|a||b|)
//for ill posed problems choose x=0
bool SafeEqSolve(Real a,Real b,Real epsilon,Real xmin,Real xmax,Real& x)
{
    if(a < 0) {
        return SafeEqSolve(-a,-b,epsilon,xmin,xmax,x);
    }

    //now we know a>=0
    Real epsScaled = epsilon*Max(a,Abs(b));

    //infinte range
    if(IsInf(xmin)==-1 && IsInf(xmax)==1) {
        if(a == 0) {
            x = 0.0;
            return Abs(b) <= epsScaled;
        }
        x = b/a;
        return true;
    }

    Real axmin=a*xmin,axmax=a*xmax;
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

bool SaveRamp(const char* fn,Real x0,Real dx0,Real x1,Real dx1, Real a,Real v,Real t)
{
    if(gSuppressSavingRamps) {
        return true;
    }
    std::string fullfilename = GetDumpDirectory();
    fullfilename += '/';
    fullfilename += fn;
    PARABOLIC_RAMP_PLOG("Saving ramp to %s\n",fullfilename.c_str());
    FILE* f=fopen(fullfilename.c_str(),"ab");
    if(!f) {
        f = fopen(fullfilename.c_str(),"wb");
        if(!f) {
            PARABOLIC_RAMP_PLOG("Unable to open file %s for saving\n",fullfilename.c_str());
            return false;
        }
    }
    double vals[7]={x0,dx0,x1,dx1,a,v,t};
    fwrite(vals,sizeof(double),7,f);
    fclose(f);
    return true;
}

bool LoadRamp(FILE* f,Real& x0,Real& dx0,Real& x1,Real& dx1, Real& a,Real& v,Real& t)
{
    double vals[7];
    int size = fread(vals,sizeof(double),7,f);
    if(size != 7) return false;
    x0=vals[0];  dx0=vals[1];
    x1=vals[2];  dx1=vals[3];
    a=vals[4];  v=vals[5];
    t=vals[6];
    return true;
}


bool LoadRamp(const char* fn,Real& x0,Real& dx0,Real& x1,Real& dx1, Real& a,Real& v,Real& t)
{
    FILE* f=fopen(fn,"rb");
    if(!f) {
        return false;
    }
    bool res=LoadRamp(f,x0,dx0,x1,dx1,a,v,t);
    fclose(f);
    return res;
}

}
