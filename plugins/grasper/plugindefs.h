// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#ifndef OPENRAVE_PLUGINDEFS_H
#define OPENRAVE_PLUGINDEFS_H

#include <cstdio>
#include <cmath>
#include <cstdlib>

// include boost for vc++ only (to get typeof working)
#ifdef _MSC_VER
#include <boost/typeof/std/string.hpp>
#include <boost/typeof/std/vector.hpp>
#include <boost/typeof/std/list.hpp>
#include <boost/typeof/std/map.hpp>
#include <boost/typeof/std/string.hpp>

#define FOREACH(it, v) for(BOOST_TYPEOF(v)::iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(BOOST_TYPEOF(v)::iterator it = (v).begin(); it != (v).end(); )

#define FOREACHC(it, v) for(BOOST_TYPEOF(v)::const_iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC_NOINC(it, v) for(BOOST_TYPEOF(v)::const_iterator it = (v).begin(); it != (v).end(); )
#define RAVE_REGISTER_BOOST
#else

#include <string>
#include <vector>
#include <list>
#include <map>
#include <string>

#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); )

#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC

#endif

#include <stdint.h>
#include <fstream>
#include <iostream>

#include <boost/assert.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>

#ifdef _MSC_VER
#define PRIdS "Id"
#else
#define PRIdS "zd"
#endif

using namespace std;

#include <sys/timeb.h>    // ftime(), struct timeb

#ifndef _WIN32
#include <sys/time.h>
#define Sleep(milli) usleep(1000*milli)
#else
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#endif

template<class T>
inline T CLAMP_ON_RANGE(T value, T min, T max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

inline uint32_t timeGetTime()
{
#ifdef _WIN32
    _timeb t;
    _ftime(&t);
#else
    timeb t;
    ftime(&t);
#endif

    return (uint32_t)(t.time*1000+t.millitm);
}

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

inline uint64_t GetMicroTime()
{
#ifdef _WIN32
    LARGE_INTEGER count, freq;
    QueryPerformanceCounter(&count);
    QueryPerformanceFrequency(&freq);
    return (count.QuadPart * 1000000) / freq.QuadPart;
#else
    struct timeval t;
    gettimeofday(&t, NULL);
    return (uint64_t)t.tv_sec*1000000+t.tv_usec;
#endif
}

struct null_deleter
{
    void operator()(void const *) const {}
};

#include <rave/rave.h>
using namespace OpenRAVE;

/// sets a new collision checker and resets to the old when destroyed
class CollisionCheckerMngr
{
 public:
    CollisionCheckerMngr(EnvironmentBasePtr penv, const string& collisionchecker) : _penv(penv)
    {
        _pprevchecker = _penv->GetCollisionChecker();
        _coloptions = _pprevchecker->GetCollisionOptions();

        if( collisionchecker.size() > 0 ) {
            _pnewchecker = _penv->CreateCollisionChecker(collisionchecker);
            if( !!_pnewchecker ) {
                RAVELOG_VERBOSEA(str(boost::format("setting collision checker %s\n")%collisionchecker));
                _penv->SetCollisionChecker(_pnewchecker);
            }
        }
    }
    ~CollisionCheckerMngr() {
        _penv->SetCollisionChecker(_pprevchecker);
        _pprevchecker->SetCollisionOptions(_coloptions);
    }
 private:
    EnvironmentBasePtr _penv;
    CollisionCheckerBasePtr _pnewchecker;
    CollisionCheckerBasePtr _pprevchecker;
    int _coloptions;
};

class GraspParameters : public PlannerBase::PlannerParameters
{
 public:
 GraspParameters(EnvironmentBasePtr penv) : fstandoff(0), ftargetroll(0), vtargetdirection(0,0,1), btransformrobot(false), breturntrajectory(false), bonlycontacttarget(true), btightgrasp(false), fcoarsestep(0.1f), ffinestep(0.001f), ftranslationstepmult(0.1f), _penv(penv) {}

    dReal fstandoff; ///< start closing fingers when at this distance
    KinBodyPtr targetbody; ///< the target that will be grasped, all parameters will be in this coordinate system. if not present, then below transformations are in absolute coordinate system.
    dReal ftargetroll; ///< rotate the hand about the palm normal (if one exists) by this many radians
    Vector vtargetdirection; ///< direction in target space to approach object from
    Vector vtargetposition; ///< position in target space to start approaching (if in collision with target, gets backed up)
    bool btransformrobot; ///< if true sets the base link of the robot given the above transformation parameters. If there is an active manipulator
    bool breturntrajectory; ///< if true, returns how the individual fingers moved instead of just the final grasp
    bool bonlycontacttarget; ///< if true, then grasp is successful only if contact is made with the target
    bool btightgrasp; ///< This is tricky, but basically if true will also move the basic link along the negative axes of some of the joints to get a tighter fit.
    vector<string> vavoidlinkgeometry; ///< list of links on the robot to avoid collisions with (for exmaple, sensors)

    dReal fcoarsestep;  ///< step for coarse planning (in radians)
    dReal ffinestep; ///< step for fine planning (in radians), THIS STEP MUST BE VERY SMALL OR THE COLLISION CHECKER GIVES WILDLY BOGUS RESULTS
    dReal ftranslationstepmult; ///< multiplication factor for translational movements of the hand or joints
        
 protected:
    EnvironmentBasePtr _penv; ///< environment target belongs to

    // save the extra data to XML
    virtual bool serialize(std::ostream& O) const
    {
        if( !PlannerParameters::serialize(O) )
            return false;
        O << "<fstandoff>" << fstandoff << "</fstandoff>" << endl;
        O << "<targetbody>" << (int)(!targetbody ? 0 : targetbody->GetNetworkId()) << "</targetbody>" << endl;
        O << "<ftargetroll>" << ftargetroll << "</ftargetroll>" << endl;
        O << "<vtargetdirection>" << vtargetdirection << "</vtargetdirection>" << endl;
        O << "<vtargetposition>" << vtargetposition << "</vtargetposition>" << endl;
        O << "<btransformrobot>" << btransformrobot << "</btransformrobot>" << endl;
        O << "<breturntrajectory>" << breturntrajectory << "</breturntrajectory>" << endl;
        O << "<bonlycontacttarget>" << bonlycontacttarget << "</bonlycontacttarget>" << endl;
        O << "<btightgrasp>" << btightgrasp << "</btightgrasp>" << endl;
        O << "<vavoidlinkgeometry>" << endl;
        FOREACHC(it,vavoidlinkgeometry)
            O << *it << " ";
        O << "</vavoidlinkgeometry>" << endl;
        O << "<fcoarsestep>" << fcoarsestep << "</fcoarsestep>" << endl;
        O << "<ffinestep>" << ffinestep << "</ffinestep>" << endl;
        O << "<ftranslationstepmult>" << ftranslationstepmult << "</ftranslationstepmult>" << endl;
        return !!O;
    }
 
    // called at the end of every XML tag, _ss contains the data 
    virtual bool endElement(const std::string& name)
    {
        // _ss is an internal stringstream that holds the data of the tag
        if( name == "vavoidlinkgeometry" )
            vavoidlinkgeometry = vector<string>((istream_iterator<string>(_ss)), istream_iterator<string>());
        else if( name == "fstandoff")
            _ss >> fstandoff;
        else if( name == "targetbody") {
            int id = 0;
            _ss >> id;
            targetbody = _penv->GetBodyFromNetworkId(id);
        }
        else if( name == "ftargetroll")
            _ss >> ftargetroll;
        else if( name == "vtargetdirection") {
            _ss >> vtargetdirection;
            vtargetdirection.normalize3();
        }
        else if( name == "vtargetposition")
            _ss >> vtargetposition;
        else if( name == "btransformrobot")
            _ss >> btransformrobot;
        else if( name == "breturntrajectory")
            _ss >> breturntrajectory;
        else if( name == "bonlycontacttarget")
            _ss >> bonlycontacttarget;
        else if( name == "btightgrasp" )
            _ss >> btightgrasp;
        else if( name == "fcoarsestep" )
            _ss >> fcoarsestep;
        else if( name == "ffinestep" )
            _ss >> ffinestep;
        else if( name == "ftranslationstepmult" )
            _ss >> ftranslationstepmult;
        else // give a chance for the default parameters to get processed
            return PlannerParameters::endElement(name);
        return false;
    }
};

#endif
