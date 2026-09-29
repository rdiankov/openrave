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
#include <openrave/logging.h>
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

#define _MAKEDATA(n) __itend__ ## n
#define MAKEDATA(n) _MAKEDATA(n)
#define MAKEVAR MAKEDATA(__LINE__)

#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(), MAKEVAR=(v).end(); it != MAKEVAR; (it)++)
#define FOREACH_NOINC(it, v) for(typeof((v).begin()) it = (v).begin(), MAKEVAR=(v).end(); it != MAKEVAR; )

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

#include <openrave/openravejson.h>

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
static const dReal g_fEpsilonEvalJointLimit = RavePow(g_fEpsilon,0.65);

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

class CallOnDestruction
{
public:
    CallOnDestruction(const boost::function<void()>& fn) : _fn(fn) {
    }
    ~CallOnDestruction() {
        _fn();
    }
private:
    boost::function<void()> _fn;
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
inline void SerializeRoundQuaternion(std::ostream& o, const RaveVector<T>& v)
{
    // This function is used only for serializing quaternions. Need to
    // take into account the fact that v and -v represent the same
    // rotation. Convert v to a rotation matrix instead to get a
    // unique representation. Then since the thrid column can be uniquely
    // determined given the first two, serializing only the first two
    // columns is sufficient for the purpose of hash computation.
    RaveTransformMatrix<T> t = matrixFromQuat(v);
    o << SerializationValue(t.m[0]) << " " << SerializationValue(t.m[4]) << " " << SerializationValue(t.m[8]) << " "
      << SerializationValue(t.m[1]) << " " << SerializationValue(t.m[5]) << " " << SerializationValue(t.m[9]) << " ";
}

template <class T>
inline void SerializeRound3(std::ostream& o, const RaveVector<T>& v)
{
    o << SerializationValue(v.x) << " " << SerializationValue(v.y) << " " << SerializationValue(v.z) << " ";
}

template <class T>
inline void SerializeRound(std::ostream& o, const RaveTransform<T>& t)
{
    SerializeRoundQuaternion(o,t.rot);
    SerializeRound3(o,t.trans);
}

template <class T>
inline void SerializeRound(std::ostream& o, const RaveTransformMatrix<T>& t)
{
    // Since the thrid column of the rotation matrix can be uniquely
    // determined given the first two, serializing only the first two
    // columns is sufficient for the purpose of hash computation.
    o << SerializationValue(t.m[0]) << " " << SerializationValue(t.m[4]) << " " << SerializationValue(t.m[8]) << " "
      << SerializationValue(t.m[1]) << " " << SerializationValue(t.m[5]) << " " << SerializationValue(t.m[9]) << " ";
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

inline bool IsZeroWithEpsilon3(const Vector v, dReal fEpsilon)
{
    return RaveFabs(v.x) <= fEpsilon && RaveFabs(v.y) <= fEpsilon && RaveFabs(v.z) <= fEpsilon;
}

inline bool IsZeroWithEpsilon4(const Vector v, dReal fEpsilon)
{
    return RaveFabs(v.x) <= fEpsilon && RaveFabs(v.y) <= fEpsilon && RaveFabs(v.z) <= fEpsilon && RaveFabs(v.w) <= fEpsilon;
}

inline dReal ComputeQuatDistance2(const Vector& quat0, const Vector& quat1)
{
    dReal e1 = (quat0-quat1).lengthsqr4();
    dReal e2 = (quat0+quat1).lengthsqr4();
    dReal e = e1 < e2 ? e1 : e2;
    return e;
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


/// \brief Update current info from json value. Create a new one if there is no id matched.
template<typename T>
void UpdateOrCreateInfo(const rapidjson::Value& value, std::vector<boost::shared_ptr<T> >& vInfos, dReal fUnitScale, int options)
{
    std::string id = OpenRAVE::orjson::GetStringJsonValueByKey(value, "id");
    bool isDeleted = OpenRAVE::orjson::GetJsonValueByKey<bool>(value, "__deleted__", false);
    typename std::vector<boost::shared_ptr<T> >::iterator itExistingInfo = vInfos.end();
    if (!id.empty()) {
        // only try to find old info if id is not empty
        FOREACH(itInfo, vInfos) {
            if ((*itInfo)->_id == id) {
                itExistingInfo = itInfo;
                break;
            }
        }
    }
    // here we allow items with empty id to be created because
    // when we load things from json, some id could be missing on file
    // and for the partial update case, the id should be non-empty
    if (itExistingInfo != vInfos.end()) {
        if (isDeleted) {
            vInfos.erase(itExistingInfo);
            return;
        }
        (*itExistingInfo)->DeserializeJSON(value, fUnitScale, options);
        (*itExistingInfo)->_id = id;
        return;
    }
    if (isDeleted) {
        return;
    }
    boost::shared_ptr<T> pNewInfo(new T());
    pNewInfo->DeserializeJSON(value, fUnitScale, options);
    pNewInfo->_id = id;
    vInfos.push_back(pNewInfo);
}

// Optimized version of UpdateOrCreateInfoWithNameCheck for updating/creating multiple info entries at once
// Given an iterable range, for each element attempt to either update an existing info record in vInfosOut or create a new record if no matching record can be found.
// When matching info records, we will attempt to match first on ID, and failing that on name.
// Since name fields may not all have the same key in the JSON representation, the name field to use for comparison is specified in pNameInJson.
// The unit scale and options fields are passed directly to the DeserializeJson method on the underlying info type T.
template <typename T>
void UpdateOrCreateInfosWithNameCheck(rapidjson::Value::ConstValueIterator sourceItBegin, rapidjson::Value::ConstValueIterator sourceItEnd, std::vector<boost::shared_ptr<T>>& vInfosOut, const char* pNameInJson, dReal fUnitScale, int options)
{
    // If the begin/end iterators are empty, don't bother even building the lookup table
    if (sourceItBegin == sourceItEnd) {
        return;
    }

    // In order to memoize name/id match lookup, create jump tables of name/id to vInfosOut index
    // Note that we allocate full strings rather than string views because otherwise deserializing into the existing info struct may mutate the view out from under the map
    std::unordered_map<std::string, size_t> infoIdxByName;
    std::unordered_map<std::string, size_t> infoIdxById;
    infoIdxByName.reserve(vInfosOut.size());
    infoIdxById.reserve(vInfosOut.size());
    for (size_t i = 0; i < vInfosOut.size(); i++) {
        const boost::shared_ptr<T>& info = vInfosOut[i];
        // Note that we emplace here rather than overwriting - we want to prioritize elements that area earlier in the vInfosOut array in the case of a duplicate
        // This replicates the old behaviour, which involved a sequential array scan.
        if (!info->_name.empty()) {
            infoIdxByName.emplace(info->_name, i);
        }
        if (!info->_id.empty()) {
            infoIdxById.emplace(info->_id, i);
        }
    }

    // Track whether we deleted any elements and need to garbage collect the output vector before returning
    bool didEraseElements = false;
    for (rapidjson::Value::ConstValueIterator it = sourceItBegin; it != sourceItEnd; it++) {
        // Bind the next JSON value to consider
        const rapidjson::Value& value = *it;

        // If the ID is non-empty, attempt to look up a matching entry in vInfosOut
        const std::string id = OpenRAVE::orjson::GetStringJsonValueByKey(value, "id");
        const std::string name = OpenRAVE::orjson::GetStringJsonValueByKey(value, pNameInJson);
        OpenRAVE::orjson::GetCStringJsonValueByKey(value, pNameInJson);
        size_t existingInfoIndex = -1;
        if (!id.empty()) {
            std::unordered_map<std::string, size_t>::iterator existingInfoIdxIt = infoIdxById.find(id);
            if (existingInfoIdxIt != infoIdxById.end()) {
                existingInfoIndex = existingInfoIdxIt->second;
            }
        }
        else {
            // Check if the new info has a name. If it doesn't, don't try and match it - just always create a new object
            // sometimes names can be empty, in which case, always create a new object
            if (!name.empty()) {
                std::unordered_map<std::string, size_t>::iterator existingInfoIdxIt = infoIdxByName.find(name);
                if (existingInfoIdxIt != infoIdxByName.end()) {
                    existingInfoIndex = existingInfoIdxIt->second;
                }
            }
        }

        // If this element is deleted, we need to remove any matching element from the info list
        bool isDeleted = OpenRAVE::orjson::GetJsonValueByKey<bool>(value, "__deleted__", false);
        if (existingInfoIndex != (size_t)-1) {
            if (isDeleted) {
                // Erase the referents to this element from our skip lists, but only if they actually point to this index
                // Note that here we look up using the ID/name of the _existing deserialized info_, not the input JSON.
                // This is to handle cases where the input JSON specifies only one of name/id but the other is already parsed from a previous update.
                std::unordered_map<std::string, size_t>::iterator idIt = infoIdxById.find(vInfosOut[existingInfoIndex]->_id);
                if (idIt != infoIdxById.end() && idIt->second == existingInfoIndex) {
                    infoIdxById.erase(idIt);
                }
                std::unordered_map<std::string, size_t>::iterator nameIt = infoIdxByName.find(vInfosOut[existingInfoIndex]->_name);
                if (nameIt != infoIdxByName.end() && nameIt->second == existingInfoIndex) {
                    infoIdxByName.erase(nameIt);
                }

                // Don't erase the backing item from the list immediately, since that would invalidate all the subsequent indices in our jump tables.
                // Instead just null the element, and we'll filter the whole list at the end.
                vInfosOut[existingInfoIndex].reset();
                didEraseElements = true;
                continue;
            }

            // If we are updating an element in place, we need to check if we're about to change the id/name
            // If we are, we have to update the jump tables.
            const bool didChangeName = !name.empty() && vInfosOut[existingInfoIndex]->_name != name; // If the name to be loaded is empty, it won't clobber anything, so don't count that as a change
            const bool didChangeId = vInfosOut[existingInfoIndex]->_id != id; // Id always overwrites, so count it as changed even if the new id is empty

            // Erase the old id/name mapping if it changed
            if (didChangeId) {
                infoIdxById.erase(vInfosOut[existingInfoIndex]->_id);
            }
            if (didChangeName) {
                infoIdxByName.erase(vInfosOut[existingInfoIndex]->_name);
            }

            // Deserialize the new json over the existing entry
            vInfosOut[existingInfoIndex]->DeserializeJSON(value, fUnitScale, options);
            vInfosOut[existingInfoIndex]->_id = id;

            // Write back the new id/name mappings with the new values if changed
            if (didChangeId) {
                infoIdxById.emplace(vInfosOut[existingInfoIndex]->_id, existingInfoIndex);
            }
            if (didChangeName) {
                infoIdxByName.emplace(vInfosOut[existingInfoIndex]->_name, existingInfoIndex);
            }

            continue;
        }

        // If the item is deleted and didn't match an existing index, then there is nothing to remove.
        // Just skip this entry.
        if (isDeleted) {
            continue;
        }

        // If we didn't match an existing entry, then create a new one now
        boost::shared_ptr<T> pNewInfo(new T());
        pNewInfo->DeserializeJSON(value, fUnitScale, options);
        pNewInfo->_id = id;

        // Update our jump tables and push this info onto the list
        const size_t newInfoIndex = vInfosOut.size();
        infoIdxById.emplace(id, newInfoIndex);
        infoIdxByName.emplace(pNewInfo->_name, newInfoIndex);
        vInfosOut.emplace_back(std::move(pNewInfo));
    }

    // If we deleted any elements, scan through and ensure there are no holes in the output before returning.
    // We iterate through the vector with two pointers; a write index and read index,
    // and coalesce all valid pointers at the start of the array while preserving order.
    // Once we're done, truncate the list to drop all the invalid entries.
    if (didEraseElements) {
        size_t readIndex = 0;
        size_t writeIndex = 0;
        while (readIndex < vInfosOut.size()) {
            // Is the read index a valid pointer?
            if (!vInfosOut[readIndex]) {
                // If item under the read index is _not_ valid, then we have a hole.
                // Increment the read index, but _don't_ increment the write index.
                readIndex++;
                continue;
            }

            // If the read and write indexes are divergent (i.e, there has been a write hole), then move this object down.
            // If the list is already contiguous, no need to shuffle (we'd just be moving objects into themselves)
            if (readIndex != writeIndex) {
                vInfosOut[writeIndex] = std::move(vInfosOut[readIndex]);
            }

            // Since this element was valid, increment both the write and read pointers.
            writeIndex++;
            readIndex++;
            continue;
        }

        // Resize to crop the end of the list
        vInfosOut.resize(writeIndex);
    }
}

template<typename T>
void UpdateOrCreateInfoWithNameCheck(const rapidjson::Value& value, std::vector<boost::shared_ptr<T> >& vInfos, const char* pNameInJson, dReal fUnitScale, int options)
{
    std::string id = OpenRAVE::orjson::GetStringJsonValueByKey(value, "id");
    bool isDeleted = OpenRAVE::orjson::GetJsonValueByKey<bool>(value, "__deleted__", false);
    typename std::vector<boost::shared_ptr<T> >::iterator itExistingInfo = vInfos.end();
    if (!id.empty()) {
        // only try to find old info if id is not empty
        FOREACH(itInfo, vInfos) {
            if ((*itInfo)->_id == id) {
                itExistingInfo = itInfo;
                break;
            }
        }
    }
    else {
        // sometimes names can be empty, in which case, always create a new object
        std::string name = OpenRAVE::orjson::GetStringJsonValueByKey(value, pNameInJson);
        if( !name.empty() ) {
            // only try to find old info if id is not empty
            FOREACH(itInfo, vInfos) {
                if ((*itInfo)->GetName() == name) {
                    itExistingInfo = itInfo;
                    id = (*itInfo)->_id;
                    break;
                }
            }
        }
    }

    // here we allow items with empty id to be created because
    // when we load things from json, some id could be missing on file
    // and for the partial update case, the id should be non-empty
    if (itExistingInfo != vInfos.end()) {
        if (isDeleted) {
            vInfos.erase(itExistingInfo);
            return;
        }
        (*itExistingInfo)->DeserializeJSON(value, fUnitScale, options);
        (*itExistingInfo)->_id = id;
        return;
    }
    if (isDeleted) {
        return;
    }
    boost::shared_ptr<T> pNewInfo(new T());
    pNewInfo->DeserializeJSON(value, fUnitScale, options);
    pNewInfo->_id = id;
    vInfos.push_back(pNewInfo);
}

/// \brief Recursively call UpdateFromInfo on children. If children need to be added or removed, require re-init. Returns false if update fails and caller should not continue with other parts of the update.
template<typename InfoPtrType, typename PtrType>
bool UpdateChildrenFromInfo(const std::vector<InfoPtrType>& vInfos, std::vector<PtrType>& vPointers, UpdateFromInfoResult& result)
{
    int index = 0;
    for (typename std::vector<InfoPtrType>::const_iterator itInfo = vInfos.begin(); itInfo != vInfos.end(); ++itInfo, ++index) {
        const InfoPtrType pInfo = *itInfo;
        PtrType pMatchExistingPointer;

        {
            typename std::vector<PtrType>::iterator itExistingSameId = vPointers.end();
            typename std::vector<PtrType>::iterator itExistingSameName = vPointers.end();
            typename std::vector<PtrType>::iterator itExistingSameIdName = vPointers.end();
            typename std::vector<PtrType>::iterator itExistingNoIdName = vPointers.end();

            // search only in the unprocessed part of vPointers
            if( (int)vPointers.size() > index ) {
                for (typename std::vector<PtrType>::iterator itPointer = vPointers.begin() + index; itPointer != vPointers.end(); ++itPointer) {
                    // special case: no id or name, find next existing one that has no id or name
                    if (pInfo->GetId().empty() && pInfo->GetName().empty()) {
                        if ((*itPointer)->GetId().empty() && (*itPointer)->GetName().empty()) {
                            itExistingNoIdName = itPointer;
                            break;
                        }
                        continue;
                    }

                    bool bIdMatch = !(*itPointer)->GetId().empty() && (*itPointer)->GetId() == pInfo->GetId();
                    bool bNameMatch = !(*itPointer)->GetName().empty() && (*itPointer)->GetName() == pInfo->GetName();
                    if( bIdMatch && bNameMatch ) {
                        itExistingSameIdName = itPointer;
                        itExistingSameId = itPointer;
                        itExistingSameName = itPointer;
                        break;
                    }
                    if( bIdMatch && itExistingSameId == vPointers.end() ) {
                        itExistingSameId = itPointer;
                    }
                    if( bNameMatch && itExistingSameName == vPointers.end() ) {
                        itExistingSameName = itPointer;
                    }
                }
            }
            typename std::vector<PtrType>::iterator itExisting = itExistingSameIdName;
            if( itExisting == vPointers.end() ) {
                itExisting = itExistingSameId;
            }
            if( itExisting == vPointers.end() ) {
                itExisting = itExistingSameName;
            }
            if( itExisting == vPointers.end() ) {
                itExisting = itExistingNoIdName;
            }
            if( itExisting != vPointers.end() ) {
                pMatchExistingPointer = *itExisting;
                if (index != itExisting-vPointers.begin()) {
                    // re-arrange vPointers according to the order of infos
                    PtrType pTemp = vPointers[index];
                    vPointers[index] = pMatchExistingPointer;
                    *itExisting = pTemp;
                }
            }
        }
        if (!pMatchExistingPointer) {
            // new element, requires re-init
            RAVELOG_VERBOSE("could not find existing pointer which matches");
            result = UFIR_RequireReinitialize;
            return false;
        }

        if( !pInfo->_id.empty() && pInfo->_id != pMatchExistingPointer->GetId() ) {
            // new element, requires re-init
            RAVELOG_VERBOSE("could not find existing pointer which matches and can update");
            result = UFIR_RequireReinitialize;
            return false;
        }

        UpdateFromInfoResult updateFromInfoResult = pMatchExistingPointer->UpdateFromInfo(*pInfo);
        if (updateFromInfoResult == UFIR_NoChange) {
            // no change
            continue;
        }

        if (updateFromInfoResult == UFIR_Success) {
            // something changd
            result = UFIR_Success;
            continue;
        }

        // update failed
        result = updateFromInfoResult;
        return false;
    }

    if (vPointers.size() > vInfos.size()) {
        // have to delete extra, require re-init
        RAVELOG_VERBOSE("current data has more elements than new data");
        result = UFIR_RequireReinitialize;
        return false;
    }

    return true;
}

template<typename T>
bool AreSharedPtrsDeepEqual(const boost::shared_ptr<T>& pFirst, const boost::shared_ptr<T>& pSecond) {
    return (pFirst == pSecond) || (!!pFirst && !!pSecond && *pFirst == *pSecond);
}

template<typename T>
bool AreVectorsDeepEqual(const std::vector<boost::shared_ptr<T> >& vFirst, const std::vector<boost::shared_ptr<T> >& vSecond) {
    if (vFirst.size() != vSecond.size()) {
        return false;
    }
    for (size_t index = 0; index < vFirst.size(); index++) {
        if( !AreSharedPtrsDeepEqual(vFirst[index], vSecond[index]) ) {
            return false;
        }
    }
    return true;
}

template<typename T, std::size_t N>
bool AreArraysDeepEqual(const boost::array<boost::shared_ptr<T>, N>& vFirst, const boost::array<boost::shared_ptr<T>, N>& vSecond) {
    for (size_t index = 0; index < vFirst.size(); index++) {
        if( !AreSharedPtrsDeepEqual(vFirst[index], vSecond[index]) ) {
            return false;
        }
    }
    return true;
}

/// \brief copies rapidjson document pointer and data from one to another. if source pointer is nullptr, then resets destination pointer to nullptr
inline void CopyRapidJsonDoc(const rapidjson::Value& source, rapidjson::Document& dest)
{
    dest = rapidjson::Document(); // to reset the allocator
    dest.CopyFrom(source, dest.GetAllocator());
}

OPENRAVE_API int64_t ConvertIsoFormatDateTimeToLinuxTimeUS(const char* pIsoFormatDateTime);

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
