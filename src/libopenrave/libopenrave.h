// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
/** \file   libopenrave.h
    \brief  Defines the private headers for libopenrave
 */

#ifndef RAVE_LIBOPENRAVE_H
#define RAVE_LIBOPENRAVE_H

#include <openrave/openrave.h> // should be included first in order to get boost throwing openrave exceptions
#include <openrave/utils.h>

//#include <boost/math/special_functions/round.hpp>

// include boost for vc++ only (to get typeof working)
#ifdef _MSC_VER
#include <boost/typeof/std/string.hpp>
#include <boost/typeof/std/vector.hpp>
#include <boost/typeof/std/list.hpp>
#include <boost/typeof/std/map.hpp>
#include <boost/typeof/std/set.hpp>
#include <boost/typeof/std/string.hpp>

#define FOREACH(it, v) for(BOOST_TYPEOF(v) ::iterator it = (v).begin(), __itend__=(v).end(); it != __itend__; ++(it))
#define FOREACH_NOINC(it, v) for(BOOST_TYPEOF(v) ::iterator it = (v).begin(), __itend__=(v).end(); it != __itend__; )

#define FOREACHC(it, v) for(BOOST_TYPEOF(v) ::const_iterator it = (v).begin(), __itend__=(v).end(); it != __itend__; ++(it))
#define FOREACHC_NOINC(it, v) for(BOOST_TYPEOF(v) ::const_iterator it = (v).begin(), __itend__=(v).end(); it != __itend__; )
#define RAVE_REGISTER_BOOST

#else
#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>
#include <algorithm>
#include <complex>

#define FOREACH(it, v) for(typeof((v).begin())it = (v).begin(), __itend__=(v).end(); it != __itend__; (it)++)
#define FOREACH_NOINC(it, v) for(typeof((v).begin())it = (v).begin(), __itend__=(v).end(); it != __itend__; )

#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC

#endif

//template <typename T>
//class openraveconst_iteratorbegin : public T::const_iterator
//{
//public:
//    openraveconst_iteratorbegin(const T & v) : T::const_iterator(v.begin()), _v(v) {
//    }
//    const T & _v;
//};
//
//
//template <typename T>
//class openraveiteratorbegin : public T::iterator
//{
//public:
//    openraveiteratorbegin(const T & v) : T::iterator( const_cast<T&> (v).begin()), _v(v) {
//    }
//    const T & _v;
//};

//#define OPENRAVE_FOREACH(it,v) for( OpenRAVE::openraveiteratorbegin<typeof(v)> (it) (v); (it) != (it)._v.end(); (it)++ )
//#define OPENRAVE_FOREACHC(it,v) for( OpenRAVE::openraveconst_iteratorbegin<typeof(v)> (it) (v); (it) != (it)._v.end(); (it)++ )

#include <stdint.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>

#ifdef USE_CRLIBM
#include <crlibm.h> // robust/accurate math
#endif

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); ++(it))

#ifdef _WIN32
#elif defined(__APPLE_CC__)
#define _strnicmp strncasecmp
#define _stricmp strcasecmp
#else
#define _strnicmp strncasecmp
#define _stricmp strcasecmp

#endif

#include <boost/bind.hpp>
#include <boost/version.hpp>

#ifdef HAVE_BOOST_FILESYSTEM
#include <boost/filesystem/operations.hpp>

#if !defined(BOOST_FILESYSTEM_VERSION) || BOOST_FILESYSTEM_VERSION <= 2
namespace boost {
namespace filesystem {
inline path absolute(const path& p)
{
    return complete(p, initial_path());
}

inline path absolute(const path& p, const path& base)
{
    return complete(p, base);
}
}
}
#endif

#endif

namespace OpenRAVE {

static const dReal g_fEpsilonLinear = RavePow(g_fEpsilon,0.9);
static const dReal g_fEpsilonJointLimit = RavePow(g_fEpsilon,0.8);
static const dReal g_fEpsilonEvalJointLimit = RavePow(g_fEpsilon,0.7);

template <typename T>
class TransformSaver
{
public:
    TransformSaver(T plink) : _plink(plink) {
        _t = _plink->GetTransform();
    }
    virtual ~TransformSaver() {
        _plink->SetTransform(_t);
    }
    const Transform& GetTransform() {
        return _t;
    }
private:
    T _plink;
    Transform _t;
};

class LinkEnableSaver
{
public:
    LinkEnableSaver(KinBody::LinkPtr plink) : _plink(plink) {
        _bIsEnabled = _plink->IsEnabled();
    }
    virtual ~LinkEnableSaver()
    {
        _plink->Enable(_bIsEnabled);
    }

private:
    KinBody::LinkPtr _plink;
    bool _bIsEnabled;
};

#define SERIALIZATION_PRECISION 4
template<typename T>
inline T SerializationValue(T f)
{
    return ( f > -1e-4f && f < 1e-4f ) ? static_cast<T>(0) : f; //boost::math::round(10000*f)*0.0001;
}

inline void SerializeRound(std::ostream& o, float f)
{
    o << SerializationValue(f) << " ";
}

inline void SerializeRound(std::ostream& o, double f)
{
    o << SerializationValue(f) << " ";
}

template <class T>
inline void SerializeRound(std::ostream& o, const RaveVector<T>& v)
{
    o << SerializationValue(v.x) << " " << SerializationValue(v.y) << " " << SerializationValue(v.z) << " " << SerializationValue(v.w) << " ";
}

template <class T>
inline void SerializeRound3(std::ostream& o, const RaveVector<T>& v)
{
    o << SerializationValue(v.x) << " " << SerializationValue(v.y) << " " << SerializationValue(v.z) << " ";
}

template <class T>
inline void SerializeRound(std::ostream& o, const RaveTransform<T>& t)
{
    // because we're serializing a quaternion, have to fix what side of the hypershpere it is on
    Vector v = t.rot;
    for(int i = 0; i < 4; ++i) {
        if( v[i] < g_fEpsilon ) {
            v = -v;
            break;
        }
        else if( v[i] > g_fEpsilon ) {
            break;
        }
    }
    SerializeRound(o,v);
    SerializeRound(o,t.trans);
}

template <class T>
inline void SerializeRound(std::ostream& o, const RaveTransformMatrix<T>& t)
{
    o << SerializationValue(t.m[0]) << " " << SerializationValue(t.m[4]) << " " << SerializationValue(t.m[8]) << " "
      << SerializationValue(t.m[1]) << " " << SerializationValue(t.m[5]) << " " << SerializationValue(t.m[9]) << " "
      << SerializationValue(t.m[2]) << " " << SerializationValue(t.m[6]) << " " << SerializationValue(t.m[10]) << " ";
    SerializeRound(o,t.trans);
}

inline int CountCircularBranches(dReal angle)
{
    if( angle > PI ) {
        return static_cast<int>((angle+PI)/(2*PI));
    }
    else if( angle < -PI ) {
        return static_cast<int>((angle-PI)/(2*PI));
    }
    return 0;
}

/// returns a value=angle+2*PI*N such that value is closest to testvalue
inline dReal GetClosestValueAlongCircle(dReal angle, dReal testvalue)
{
    int n = static_cast<int>((testvalue-angle)/PI);
    if( n >= 1 ) {
        return angle + static_cast<dReal>((n+1)/2)*2*PI;
    }
    else if( n <= -1 ) {
        return angle + static_cast<dReal>((n-1)/2)*2*PI;
    }
    return angle;
}

inline dReal TransformDistanceFast(const Transform& t1, const Transform& t2, dReal frotweight=1, dReal ftransweight=1)
{
    dReal e1 = (t1.rot-t2.rot).lengthsqr4();
    dReal e2 = (t1.rot+t2.rot).lengthsqr4();
    dReal e = e1 < e2 ? e1 : e2;
    return RaveSqrt((t1.trans-t2.trans).lengthsqr3() + frotweight*e);
}

inline dReal TransformDistance2(const Transform& t1, const Transform& t2, dReal frotweight=1, dReal ftransweight=1)
{
    //dReal facos = RaveAcos(min(dReal(1),RaveFabs(dot4(t1.rot,t2.rot))));
    dReal fcos1 = (t1.rot-t2.rot).lengthsqr4();
    dReal fcos2 = (t1.rot+t2.rot).lengthsqr4();
    dReal fcos = fcos1 < fcos2 ? fcos1 : fcos2;
    return (t1.trans-t2.trans).lengthsqr3() + frotweight*fcos; //*fcos;
}

int SetDOFValuesIndicesParameters(KinBodyPtr pbody, const std::vector<dReal>& values, const std::vector<int>& vindices, int options);
int SetDOFVelocitiesIndicesParameters(KinBodyPtr pbody, const std::vector<dReal>& velocities, const std::vector<int>& vindices, int options);
int CallSetStateValuesFns(const std::vector< std::pair<PlannerBase::PlannerParameters::SetStateValuesFn, int> >& vfunctions, int nDOF, int nMaxDOFForGroup, const std::vector<dReal>& v, int options);

void CallGetStateFns(const std::vector< std::pair<PlannerBase::PlannerParameters::GetStateFn, int> >& vfunctions, int nDOF, int nMaxDOFForGroup, std::vector<dReal>& v);

void subtractstates(std::vector<dReal>& q1, const std::vector<dReal>& q2);

/// \brief The information of a currently grabbed body.
class Grabbed : public UserData, public boost::enable_shared_from_this<Grabbed>
{
public:
    Grabbed(KinBodyPtr pgrabbedbody, KinBody::LinkPtr plinkrobot) : _pgrabbedbody(pgrabbedbody), _plinkrobot(plinkrobot) {
        _enablecallback = pgrabbedbody->RegisterChangeCallback(KinBody::Prop_LinkEnable, boost::bind(&Grabbed::UpdateCollidingLinks, this));
        _plinkrobot->GetRigidlyAttachedLinks(_vattachedlinks);
    }
    virtual ~Grabbed() {
    }
    KinBodyWeakPtr _pgrabbedbody;         ///< the grabbed body
    KinBody::LinkPtr _plinkrobot;         ///< robot link that is grabbing the body
    std::list<KinBody::LinkConstPtr> _listNonCollidingLinks;         ///< links that are not colliding with the grabbed body at the time of Grab
    Transform _troot;         ///< root transform (of first link of body) relative to plinkrobot's transform. In other words, pbody->GetTransform() == plinkrobot->GetTransform()*troot
    std::set<int> _setRobotLinksToIgnore; ///< original links of the robot to force ignoring

    /// \brief check collision with all links to see which are valid.
    ///
    /// Use the robot's self-collision checker if possible
    /// resets all cached data and re-evaluates the collisions
    /// \param setRobotLinksToIgnore indices of the robot links to always ignore, in other words remove from non-colliding list
    void _ProcessCollidingLinks(const std::set<int>& setRobotLinksToIgnore);

    inline const std::vector<KinBody::LinkPtr>& GetRigidlyAttachedLinks() const {
        return _vattachedlinks;
    }

    void AddMoreIgnoreLinks(const std::set<int>& setRobotLinksToIgnore)
    {
        RobotBasePtr probot = RaveInterfaceCast<RobotBase>(_plinkrobot->GetParent());
        FOREACHC(itignoreindex, setRobotLinksToIgnore) {
            _setRobotLinksToIgnore.insert(*itignoreindex);
            KinBody::LinkPtr plink = probot->GetLinks().at(*itignoreindex);
            _mapLinkIsNonColliding[plink] = 0;
            _listNonCollidingLinks.remove(plink);
        }
    }

    /// return -1 for unknown, 0 for no, 1 for yes
    int WasLinkNonColliding(KinBody::LinkConstPtr plink) const
    {
        std::map<KinBody::LinkConstPtr, int>::const_iterator it = _mapLinkIsNonColliding.find(plink);
        if( it != _mapLinkIsNonColliding.end() ) {
            return it->second;
        }
        return -1;
    }

    /// \brief updates the non-colliding info while reusing the cache data from _ProcessCollidingLinks
    ///
    /// note that Regrab here is *very* dangerous since the robot could be a in a bad self-colliding state with the body. therefore, update the non-colliding state based on _mapLinkIsNonColliding
    void UpdateCollidingLinks()
    {
        RobotBasePtr probot = RaveInterfaceCast<RobotBase>(_plinkrobot->GetParent());
        if( !probot ) {
            return;
        }
        EnvironmentBasePtr penv = probot->GetEnv();
        KinBodyConstPtr pgrabbedbody(_pgrabbedbody);
        if( !pgrabbedbody || !pgrabbedbody->IsEnabled() ) {
            _listNonCollidingLinks.clear();
            return;
        }

        CollisionOptionsStateSaverPtr colsaver;
        CollisionCheckerBasePtr pchecker = probot->GetSelfCollisionChecker();
        if( !pchecker ) {
            pchecker = penv->GetCollisionChecker();
        }

        std::map<KinBody::LinkConstPtr, int>::iterator itnoncolliding;
        std::vector<KinBody::LinkPtr > vbodyattachedlinks;
        FOREACHC(itgrabbed, probot->_vGrabbedBodies) {
            boost::shared_ptr<Grabbed const> pgrabbed = boost::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
            bool bsamelink = find(_vattachedlinks.begin(),_vattachedlinks.end(), pgrabbed->_plinkrobot) != _vattachedlinks.end();
            KinBodyPtr pothergrabbedbody(pgrabbed->_pgrabbedbody);
            if( !!pothergrabbedbody && pothergrabbedbody != pgrabbedbody && pothergrabbedbody->GetLinks().size() > 0 ) {
                if( bsamelink ) {
                    pothergrabbedbody->GetLinks().at(0)->GetRigidlyAttachedLinks(vbodyattachedlinks);
                }
                KinBody::KinBodyStateSaverPtr othergrabbedbodysaver;
                FOREACHC(itgrabbedlink, pothergrabbedbody->GetLinks()) {
                    itnoncolliding = _mapLinkIsNonColliding.find(*itgrabbedlink);
                    if( itnoncolliding == _mapLinkIsNonColliding.end() ) {
                        if( bsamelink && find(vbodyattachedlinks.begin(),vbodyattachedlinks.end(), *itgrabbedlink) != vbodyattachedlinks.end() ) {
                            _mapLinkIsNonColliding[*itgrabbedlink] = 0;
                        }
                        else {
                            // new body?
                            if( !colsaver ) {
                                // have to reset the collision options
                                colsaver.reset(new CollisionOptionsStateSaver(pchecker,0));
                            }
                            if( !othergrabbedbodysaver ) {
                                othergrabbedbodysaver.reset(new KinBody::KinBodyStateSaver(pothergrabbedbody, KinBody::Save_LinkEnable));
                                pothergrabbedbody->Enable(true);
                            }
                            _mapLinkIsNonColliding[*itgrabbedlink] = !pchecker->CheckCollision(KinBody::LinkConstPtr(*itgrabbedlink), pgrabbedbody);
                        }
                    }
                }
            }
        }

        std::set<KinBodyConstPtr> _setgrabbed;
        FOREACHC(itgrabbed, probot->_vGrabbedBodies) {
            boost::shared_ptr<Grabbed const> pgrabbed = boost::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
            KinBodyConstPtr pothergrabbedbody(pgrabbed->_pgrabbedbody);
            if( !!pothergrabbedbody ) {
                _setgrabbed.insert(pothergrabbedbody);
            }
        }

        _listNonCollidingLinks.clear();
        itnoncolliding = _mapLinkIsNonColliding.begin();
        while( itnoncolliding != _mapLinkIsNonColliding.end() ) {
            KinBodyPtr noncollidingparent = itnoncolliding->first->GetParent(true);
            if( !noncollidingparent ) {
                _mapLinkIsNonColliding.erase(itnoncolliding++);
                continue;
            }
            if( noncollidingparent != probot ) {
                // check if body is currently being grabbed
                if( _setgrabbed.find(noncollidingparent) == _setgrabbed.end() ) {
                    _mapLinkIsNonColliding.erase(itnoncolliding++);
                    continue;
                }
            }

            if( itnoncolliding->second && itnoncolliding->first->IsEnabled() ) {
                _listNonCollidingLinks.push_back(itnoncolliding->first);
            }
            ++itnoncolliding;
        }
    }

private:
    std::vector<KinBody::LinkPtr> _vattachedlinks;
    UserDataPtr _enablecallback; ///< callback for grabbed body when it is enabled/disabled

    std::map<KinBody::LinkConstPtr, int> _mapLinkIsNonColliding; // the collision state for each link at the time the body was grabbed.
};

typedef boost::shared_ptr<Grabbed> GrabbedPtr;
typedef boost::shared_ptr<Grabbed const> GrabbedConstPtr;

/// -1 v1 is smaller than v2
// 0 two vectors are equivalent
/// +1 v1 is greater than v2
inline int CompareRealVectors(const std::vector<dReal> & v1, const std::vector<dReal>& v2, dReal epsilon)
{
    if( v1.size() != v2.size() ) {
        return v1.size() < v2.size() ? -1 : 1;
    }
    for(size_t i = 0; i < v1.size(); ++i) {
        if( v1[i] < v2[i]-epsilon ) {
            return -1;
        }
        else if( v1[i] > v2[i]+epsilon ) {
            return 1;
        }
    }
    return 0;
}

template <typename IKReal>
inline void polyroots2(const IKReal* rawcoeffs, IKReal* rawroots, int& numroots)
{
    IKReal det = rawcoeffs[1]*rawcoeffs[1]-4*rawcoeffs[0]*rawcoeffs[2];
    if( det < 0 ) {
        numroots=0;
    }
    else if( det == 0 ) {
        rawroots[0] = -0.5*rawcoeffs[1]/rawcoeffs[0];
        numroots = 1;
    }
    else {
        det = RaveSqrt(det);
        rawroots[0] = (-rawcoeffs[1]+det)/(2*rawcoeffs[0]);
        rawroots[1] = (-rawcoeffs[1]-det)/(2*rawcoeffs[0]); //rawcoeffs[2]/(rawcoeffs[0]*rawroots[0]);
        numroots = 2;
    }
}

/// \brief Durand-Kerner polynomial root finding method
template <typename IKReal, int D>
inline void polyroots(const IKReal* rawcoeffs, IKReal* rawroots, int& numroots)
{
    using std::complex;
    BOOST_ASSERT(rawcoeffs[0] != 0);
    const IKReal tol = 128.0*std::numeric_limits<IKReal>::epsilon();
    const IKReal tolsqrt = sqrt(std::numeric_limits<IKReal>::epsilon());
    complex<IKReal> coeffs[D];
    const int maxsteps = 110;
    for(int i = 0; i < D; ++i) {
        coeffs[i] = complex<IKReal>(rawcoeffs[i+1]/rawcoeffs[0]);
    }
    complex<IKReal> roots[D];
    IKReal err[D];
    roots[0] = complex<IKReal>(1,0);
    roots[1] = complex<IKReal>(0.4,0.9); // any complex number not a root of unity works
    err[0] = 1.0;
    err[1] = 1.0;
    for(int i = 2; i < D; ++i) {
        roots[i] = roots[i-1]*roots[1];
        err[i] = 1.0;
    }
    for(int step = 0; step < maxsteps; ++step) {
        bool changed = false;
        for(int i = 0; i < D; ++i) {
            if ( err[i] >= tol ) {
                changed = true;
                // evaluate
                complex<IKReal> x = roots[i] + coeffs[0];
                for(int j = 1; j < D; ++j) {
                    x = roots[i] * x + coeffs[j];
                }
                for(int j = 0; j < D; ++j) {
                    if( i != j ) {
                        if( roots[i] != roots[j] ) {
                            x /= (roots[i] - roots[j]);
                        }
                    }
                }
                roots[i] -= x;
                err[i] = abs(x);
            }
        }
        if( !changed ) {
            break;
        }
    }

    numroots = 0;
    bool visited[D] = {false};
    for(int i = 0; i < D; ++i) {
        if( !visited[i] ) {
            // might be a multiple root, in which case it will have more error than the other roots
            // find any neighboring roots, and take the average
            complex<IKReal> newroot=roots[i];
            int n = 1;
            for(int j = i+1; j < D; ++j) {
                if( abs(roots[i]-roots[j]) < 8*tolsqrt ) {
                    newroot += roots[j];
                    n += 1;
                    visited[j] = true;
                }
            }
            if( n > 1 ) {
                newroot /= n;
            }
            // there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
            if( RaveFabs(imag(newroot)) < tolsqrt ) {
                rawroots[numroots++] = real(newroot);
            }
        }
    }
}

namespace LocalXML
{
bool ParseXMLData(BaseXMLReaderPtr preader, const char* buffer, int size);
}

#ifdef _WIN32
inline const char *strcasestr(const char *s, const char *find)
{
    register char c, sc;
    register size_t len;

    if ((c = *find++) != 0) {
        c = tolower((unsigned char)c);
        len = strlen(find);
        do {
            do {
                if ((sc = *s++) == 0) {
                    return (NULL);
                }
            } while ((char)tolower((unsigned char)sc) != c);
        } while (_strnicmp(s, find, len) != 0);
        s--;
    }
    return ((char *) s);
}
#endif

} // end OpenRAVE namespace

// need the prototypes in order to keep them free of the OpenRAVE namespace
namespace OpenRAVEXMLParser
{
class InterfaceXMLReader;
class KinBodyXMLReader;
class LinkXMLReader;
class JointXMLReader;
class ManipulatorXMLReader;
class AttachedSensorXMLReader;
class RobotXMLReader;
class EnvironmentXMLReader;
}
class Environment;

using namespace OpenRAVE;
using namespace std;

#define _(msgid) OpenRAVE::RaveGetLocalizedTextForDomain("openrave", msgid)
#endif
