// -*- coding: utf-8 -*-
// Copyright (C) 2006-2023
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
#include "libopenrave.h"

namespace OpenRAVE {

//bool CompareCollisionPairInfo(const CollisionPairInfo& pairContact1, const CollisionPairInfo& pairContact2)
//{
//    int cmp = pairContact1.bodyLinkGeom1Name.compare(pairContact2.bodyLinkGeom1Name);
//    if( cmp != 0 ) {
//        return cmp < 0;
//    }
//
//    // compare pgeom2 since pgeom1 is equal
//    return pairContact1.bodyLinkGeom2Name < pairContact2.bodyLinkGeom2Name;
//}

void CONTACT::SaveToJson(rapidjson::Value& rContact, rapidjson::Document::AllocatorType& alloc) const
{
    // there can be a lot of contacts, so should be more compact without the member names
    rContact.SetArray();
    rContact.Reserve(7, alloc);
    rContact.PushBack(pos.x,alloc);
    rContact.PushBack(pos.y,alloc);
    rContact.PushBack(pos.z,alloc);
    rContact.PushBack(norm.x,alloc);
    rContact.PushBack(norm.y,alloc);
    rContact.PushBack(norm.z,alloc);
    rContact.PushBack(depth,alloc);
}

void CONTACT::LoadFromJson(const rapidjson::Value& rContact)
{
    pos = Vector();
    norm = Vector();
    depth = 0;

    if( rContact.IsNull() ) {
        return;
    }
    BOOST_ASSERT(rContact.IsArray() && rContact.Size() == 7);
    orjson::LoadJsonValue(rContact[0], pos.x);
    orjson::LoadJsonValue(rContact[1], pos.y);
    orjson::LoadJsonValue(rContact[2], pos.z);
    orjson::LoadJsonValue(rContact[3], norm.x);
    orjson::LoadJsonValue(rContact[4], norm.y);
    orjson::LoadJsonValue(rContact[5], norm.z);
    orjson::LoadJsonValue(rContact[6], depth);
}

void CollisionPairInfo::Swap(CollisionPairInfo& rhs)
{
    bodyLinkGeom1Name.swap(rhs.bodyLinkGeom1Name);
    bodyLinkGeom2Name.swap(rhs.bodyLinkGeom2Name);
    contacts.swap(rhs.contacts);
}

void CollisionPairInfo::SwapFirstSecond()
{
    std::swap(bodyLinkGeom1Name, bodyLinkGeom2Name);
    for(CONTACT& c : contacts) {
        c.norm = -c.norm;
        c.depth = -c.depth;
    }
}

void CollisionPairInfo::SetFirstCollision(const std::string& bodyname, const std::string& linkname, const std::string& geomname)
{
    bodyLinkGeom1Name = bodyname;
    bodyLinkGeom1Name.push_back(' ');
    bodyLinkGeom1Name += linkname;
    bodyLinkGeom1Name.push_back(' ');
    bodyLinkGeom1Name += geomname;
}

void CollisionPairInfo::SetSecondCollision(const std::string& bodyname, const std::string& linkname, const std::string& geomname)
{
    bodyLinkGeom2Name = bodyname;
    bodyLinkGeom2Name.push_back(' ');
    bodyLinkGeom2Name += linkname;
    bodyLinkGeom2Name.push_back(' ');
    bodyLinkGeom2Name += geomname;
}

void CollisionPairInfo::ExtractFirstBodyName(string_view& bodyname) const
{
    bodyname = string_view();
    if( !bodyLinkGeom1Name.empty() ) {
        size_t index = bodyLinkGeom1Name.find_first_of(' ');
        if( index != std::string::npos ) {
            bodyname = string_view(bodyLinkGeom1Name.c_str(), index);
        }
        else {
            bodyname = string_view(bodyLinkGeom1Name.c_str());
        }
    }
}

void CollisionPairInfo::ExtractSecondBodyName(string_view& bodyname) const
{
    bodyname = string_view();
    if( !bodyLinkGeom2Name.empty() ) {
        size_t index = bodyLinkGeom2Name.find_first_of(' ');
        if( index != std::string::npos ) {
            bodyname = string_view(bodyLinkGeom2Name.c_str(), index);
        }
        else {
            bodyname = string_view(bodyLinkGeom2Name.c_str());
        }
    }
}

void CollisionPairInfo::ExtractFirstLinkName(string_view& linkname) const
{
    linkname = string_view();
    size_t firstindex = bodyLinkGeom1Name.find_first_of(' ');
    if( firstindex == std::string::npos ) {
        return;
    }

    size_t secondindex = bodyLinkGeom1Name.find_first_of(' ', firstindex+1);
    if( secondindex == std::string::npos ) {
        linkname = string_view(bodyLinkGeom1Name.c_str()+firstindex+1, bodyLinkGeom1Name.size()-firstindex-1);
    }
    else {
        linkname = string_view(bodyLinkGeom1Name.c_str()+firstindex+1,secondindex - firstindex - 1);
    }
}

void CollisionPairInfo::ExtractSecondLinkName(string_view& linkname) const
{
    linkname = string_view();
    size_t firstindex = bodyLinkGeom2Name.find_first_of(' ');
    if( firstindex == std::string::npos ) {
        return;
    }

    size_t secondindex = bodyLinkGeom2Name.find_first_of(' ', firstindex+1);
    if( secondindex == std::string::npos ) {
        linkname = string_view(bodyLinkGeom2Name.c_str()+firstindex+1, bodyLinkGeom2Name.size()-firstindex-1);
    }
    else {
        linkname = string_view(bodyLinkGeom2Name.c_str()+firstindex+1, secondindex - firstindex - 1);
    }
}

inline void _ExtractBodyLinkNames(const std::string& bodyLinkGeomName, string_view& bodyname, string_view& linkname)
{
    bodyname = string_view();
    linkname = string_view();
    size_t firstindex = bodyLinkGeomName.find_first_of(' ');
    if( firstindex == std::string::npos ) {
        bodyname = string_view(bodyLinkGeomName.c_str());
        return;
    }

    bodyname = string_view(bodyLinkGeomName.c_str(),firstindex);

    size_t secondindex = bodyLinkGeomName.find_first_of(' ', firstindex+1);
    if( secondindex == std::string::npos ) {
        linkname = string_view(bodyLinkGeomName.c_str()+firstindex+1, bodyLinkGeomName.size()-firstindex-1);
    }
    else {
        linkname = string_view(bodyLinkGeomName.c_str()+firstindex+1, secondindex - firstindex - 1);
    }
}

void CollisionPairInfo::ExtractFirstBodyLinkNames(string_view& bodyname, string_view& linkname) const
{
    return _ExtractBodyLinkNames(bodyLinkGeom1Name, bodyname, linkname);
}

void CollisionPairInfo::ExtractSecondBodyLinkNames(string_view& bodyname, string_view& linkname) const
{
    return _ExtractBodyLinkNames(bodyLinkGeom2Name, bodyname, linkname);
}

inline void _ExtractBodyLinkGeomNames(const std::string& bodyLinkGeomName, string_view& bodyname, string_view& linkname, string_view& geomname)
{
    bodyname = string_view();
    linkname = string_view();
    geomname = string_view();
    size_t firstindex = bodyLinkGeomName.find_first_of(' ');
    if( firstindex == std::string::npos ) {
        bodyname = string_view(bodyLinkGeomName.c_str());
        return;
    }

    bodyname = string_view(bodyLinkGeomName.c_str(),firstindex);

    size_t secondindex = bodyLinkGeomName.find_first_of(' ', firstindex+1);
    if( secondindex == std::string::npos ) {
        linkname = string_view(bodyLinkGeomName.c_str()+firstindex+1, bodyLinkGeomName.size()-firstindex-1);
        return;
    }

    linkname = string_view(bodyLinkGeomName.c_str()+firstindex+1, secondindex - firstindex - 1);

    size_t thirdindex = bodyLinkGeomName.find_first_of(' ', secondindex+1);
    if( thirdindex == std::string::npos ) {
        geomname = string_view(bodyLinkGeomName.c_str()+secondindex+1, bodyLinkGeomName.size()-secondindex-1);
        return;
    }

    geomname = string_view(bodyLinkGeomName.c_str()+secondindex+1, thirdindex - secondindex - 1);
}

void CollisionPairInfo::ExtractFirstBodyLinkGeomNames(string_view& bodyname, string_view& linkname, string_view& geomname) const
{
    _ExtractBodyLinkGeomNames(bodyLinkGeom1Name, bodyname, linkname, geomname);
}

void CollisionPairInfo::ExtractSecondBodyLinkGeomNames(string_view& bodyname, string_view& linkname, string_view& geomname) const
{
    _ExtractBodyLinkGeomNames(bodyLinkGeom2Name, bodyname, linkname, geomname);
}

inline int _CompareLink(const std::string& bodyLinkGeomName, const KinBody::Link& link)
{
    // compare the linkname first since getting parent requires atomic operations
    size_t firstindex = bodyLinkGeomName.find_first_of(' ');
    if( firstindex == std::string::npos ) {
        return -1; // no link
    }

    string_view linkname;

    size_t secondindex = bodyLinkGeomName.find_first_of(' ', firstindex+1);
    if( secondindex == std::string::npos ) {
        linkname = string_view(bodyLinkGeomName.c_str()+firstindex+1, bodyLinkGeomName.size()-firstindex-1);
    }
    else {
        linkname = string_view(bodyLinkGeomName.c_str()+firstindex+1, secondindex - firstindex - 1);
    }

    int linkcmp = linkname.compare(link.GetName());
    if( linkcmp != 0 ) {
        return linkcmp;
    }

    // finally compare the body names
    return string_view(bodyLinkGeomName.c_str(), firstindex).compare(link.GetParent()->GetName());
}

int CollisionPairInfo::CompareFirstLink(const KinBody::Link& link) const
{
    return _CompareLink(bodyLinkGeom1Name, link);
}

int CollisionPairInfo::CompareSecondLink(const KinBody::Link& link) const
{
    return _CompareLink(bodyLinkGeom2Name, link);
}

int CollisionPairInfo::CompareFirstBodyName(const std::string& bodyname) const
{
    if( bodyLinkGeom1Name.size() < bodyname.size() ) {
        return -1;
    }

    return strncmp(bodyLinkGeom1Name.c_str(), bodyname.c_str(), bodyname.size());
}

int CollisionPairInfo::CompareSecondBodyName(const std::string& bodyname) const
{
    if( bodyLinkGeom2Name.size() < bodyname.size() ) {
        return -1;
    }

    return strncmp(bodyLinkGeom2Name.c_str(), bodyname.c_str(), bodyname.size());
}

inline int _FindMatchingLinkIndex(const std::string& bodyLinkGeomName, const std::vector<KinBody::LinkPtr>& vlinks)
{
    if( vlinks.size() == 0 ) {
        return -1;
    }

    size_t firstindex = bodyLinkGeomName.find_first_of(' ');
    if( firstindex == std::string::npos ) {
        return -1;
    }

    const std::string& bodyname = vlinks[0]->GetParent()->GetName();
    if( bodyname.size() != firstindex ) {
        return -1;
    }

    if( strncmp(bodyname.c_str(), bodyLinkGeomName.c_str(), firstindex) != 0 ) {
        return -1;
    }

    // body matches, now check the links
    size_t secondindex = bodyLinkGeomName.find_first_of(' ', firstindex+1);
    size_t nLinkNameLength;
    if( secondindex != std::string::npos ) {
        nLinkNameLength = secondindex - firstindex;
    }
    else {
        nLinkNameLength = bodyLinkGeomName.size() - firstindex;
    }

    for(int ilink = 0; ilink < (int)vlinks.size(); ++ilink) {
        const KinBody::LinkPtr& plink = vlinks[ilink];
        if( plink->GetName().size() == nLinkNameLength ) {
            if( strncmp(plink->GetName().c_str(), bodyLinkGeomName.c_str()+firstindex+1, nLinkNameLength) == 0 ) {
                return ilink;
            }
        }
    }

    return -1;
}

int CollisionPairInfo::FindFirstMatchingLinkIndex(const std::vector<KinBody::LinkPtr>& vlinks) const
{
    return _FindMatchingLinkIndex(bodyLinkGeom1Name, vlinks);
}

int CollisionPairInfo::FindSecondMatchingLinkIndex(const std::vector<KinBody::LinkPtr>& vlinks) const
{
    return _FindMatchingLinkIndex(bodyLinkGeom1Name, vlinks);
}

inline int _CompareLinkName(const std::string& bodyLinkGeomName, const std::string& linkname)
{
    size_t index = bodyLinkGeomName.find_first_of(' ');
    if( index == std::string::npos ) {
        return -1;
    }

    int nRemainingLength = (int)bodyLinkGeomName.size() - index - 1;
    if( nRemainingLength < (int)linkname.size() ) {
        return -1;
    }

    return strncmp(bodyLinkGeomName.c_str()+index+1, linkname.c_str(), linkname.size());
}

int CollisionPairInfo::CompareFirstLinkName(const std::string& linkname) const
{
    return _CompareLinkName(bodyLinkGeom1Name, linkname);
}
int CollisionPairInfo::CompareSecondLinkName(const std::string& linkname) const
{
    return _CompareLinkName(bodyLinkGeom2Name, linkname);
}

inline KinBodyPtr _ExtractBody(const std::string& bodyLinkGeomName, EnvironmentBase& env)
{
    if( bodyLinkGeomName.empty() ) {
        return KinBodyPtr();
    }

    size_t index = bodyLinkGeomName.find_first_of(' ');
    int nNameLength;
    if( index != std::string::npos ) {
        nNameLength = index;
    }
    else {
        nNameLength = bodyLinkGeomName.size();
    }

    return env.GetKinBody(string_view(bodyLinkGeomName.c_str(), nNameLength));
}

KinBodyPtr CollisionPairInfo::ExtractFirstBody(EnvironmentBase& env) const
{
    return _ExtractBody(bodyLinkGeom1Name, env);
}

KinBodyPtr CollisionPairInfo::ExtractSecondBody(EnvironmentBase& env) const
{
    return _ExtractBody(bodyLinkGeom2Name, env);
}

void CollisionPairInfo::SaveToJson(rapidjson::Value& rCollision, rapidjson::Document::AllocatorType& alloc) const
{
    rCollision.SetObject();
    orjson::SetJsonValueByKey(rCollision, "bodyLinkGeom1Name", bodyLinkGeom1Name, alloc);
    orjson::SetJsonValueByKey(rCollision, "bodyLinkGeom2Name", bodyLinkGeom2Name, alloc);

    rapidjson::Value rContacts;
    rContacts.SetArray();
    rContacts.Reserve(contacts.size(),alloc);
    for(const CONTACT& c : contacts) {
        rapidjson::Value rContact;
        c.SaveToJson(rContact,alloc);
        rContacts.PushBack(rContact,alloc);
    }
    rCollision.AddMember("contacts", rContacts, alloc);
}

void CollisionPairInfo::LoadFromJson(const rapidjson::Value& rCollision)
{
    Reset();
    if( rCollision.IsNull() ) {
        return;
    }

    orjson::LoadJsonValueByKey(rCollision, "bodyLinkGeom1Name", bodyLinkGeom1Name);
    orjson::LoadJsonValueByKey(rCollision, "bodyLinkGeom2Name", bodyLinkGeom2Name);

    rapidjson::Value::ConstMemberIterator itContacts = rCollision.FindMember("contacts");
    if( itContacts != rCollision.MemberEnd() ) {
        const rapidjson::Value& rContacts = itContacts->value;
        if( rContacts.IsArray() ) {
            contacts.resize(rContacts.Size());
            for(int icontact = 0; icontact < (int)contacts.size(); ++icontact) {
                contacts[icontact].LoadFromJson(rContacts[icontact]);
            }
        }
    }
}

CollisionReport::CollisionReport() {
}

CollisionReport::CollisionReport(const CollisionReport& rhs)
{
    *this = rhs;
}

void CollisionReport::Reset(int coloptions)
{
    options = coloptions;
    if( !(nKeepPrevious & 1) ) {
        minDistance = 1e20f;
        numWithinTol = 0;
        nNumValidCollisions = 0;
    }
}

void CollisionReport::Swap(CollisionReport& rhs)
{
    vCollisionInfos.swap(rhs.vCollisionInfos);
    std::swap(nNumValidCollisions, rhs.nNumValidCollisions);
    std::swap(options, rhs.options);
    std::swap(minDistance, rhs.minDistance);
    std::swap(numWithinTol, rhs.numWithinTol);
    std::swap(nKeepPrevious, rhs.nKeepPrevious);
}

CollisionReport& CollisionReport::operator=(const CollisionReport& rhs)
{
    nNumValidCollisions = rhs.nNumValidCollisions;
    if( nNumValidCollisions > (int)vCollisionInfos.size() ) {
        vCollisionInfos.resize(nNumValidCollisions);
    }
    OPENRAVE_ASSERT_OP(rhs.nNumValidCollisions,<=,(int)rhs.vCollisionInfos.size()); // sanity check
    for(int index = 0; index < nNumValidCollisions; ++index) {
        vCollisionInfos[index] = rhs.vCollisionInfos[index];
    }
    options = rhs.options;
    minDistance = rhs.minDistance;
    numWithinTol = rhs.numWithinTol;
    nKeepPrevious = rhs.nKeepPrevious;
    return *this;
}

std::string CollisionReport::__str__() const
{
    std::stringstream s;
    s << "[";
    for(int index = 0; index < nNumValidCollisions; ++index) {
        const CollisionPairInfo& cpinfo = vCollisionInfos.at(index);
        s << "(" << cpinfo.bodyLinkGeom1Name << ")x(" << cpinfo.bodyLinkGeom2Name << ")";
        if( cpinfo.contacts.size() > 0 ) {
            s << ", c=" << cpinfo.contacts.size();
        }
        s << ", ";
    }
    s << "]";
    if( minDistance < 1e10 ) {
        s << ", mindist="<<minDistance;
    }
    return s.str();
}

void CollisionReport::SaveToJson(rapidjson::Value& rCollisionReport, rapidjson::Document::AllocatorType& alloc) const
{
    rCollisionReport.SetObject();

    if( nNumValidCollisions > 0 ) {
        rapidjson::Value rCollisionInfos;
        rCollisionInfos.SetArray();
        rCollisionInfos.Reserve(nNumValidCollisions, alloc);
        for(int index = 0; index < nNumValidCollisions; ++index) {
            rapidjson::Value rCollisionInfo;
            vCollisionInfos[index].SaveToJson(rCollisionInfo, alloc);
            rCollisionInfos.PushBack(rCollisionInfo, alloc);
        }
        rCollisionReport.AddMember("collisionInfos", rCollisionInfos, alloc);
    }
    if( options ) {
        orjson::SetJsonValueByKey(rCollisionReport, "options", options, alloc);
    }
    if( minDistance < 1e10 ) {
        orjson::SetJsonValueByKey(rCollisionReport, "minDistance", minDistance, alloc);
    }
    if( numWithinTol ) {
        orjson::SetJsonValueByKey(rCollisionReport, "numWithinTol", numWithinTol, alloc);
    }
}

void CollisionReport::LoadFromJson(const rapidjson::Value& rCollisionReport)
{
    Reset();
    if( rCollisionReport.IsNull() ) {
        return;
    }

    rapidjson::Value::ConstMemberIterator itCollisionInfos = rCollisionReport.FindMember("collisionInfos");
    if( itCollisionInfos != rCollisionReport.MemberEnd() ) {
        const rapidjson::Value& rCollisionInfos = itCollisionInfos->value;
        if( rCollisionInfos.IsArray() ) {
            nNumValidCollisions = rCollisionInfos.Size();
            if( nNumValidCollisions > (int)vCollisionInfos.size() ) {
                vCollisionInfos.resize(nNumValidCollisions);
            }
            for(int icollision = 0; icollision < nNumValidCollisions; ++icollision) {
                vCollisionInfos[icollision].LoadFromJson(rCollisionInfos[icollision]);
            }
        }
    }

    orjson::LoadJsonValueByKey(rCollisionReport, "options", options);
    orjson::LoadJsonValueByKey(rCollisionReport, "minDistance", minDistance);
    orjson::LoadJsonValueByKey(rCollisionReport, "numWithinTol", numWithinTol);
}

int CollisionReport::AddCollision()
{
    // first write the collision as if it was unique
    if( nNumValidCollisions+1 >= (int)vCollisionInfos.size() ) {
        vCollisionInfos.resize(nNumValidCollisions+1);
    }
    // return the new index
    return nNumValidCollisions++;
}

int CollisionReport::AddLinkCollision(const KinBody::Link& link1)
{
    // first write the collision as if it was unique
    if( nNumValidCollisions+1 >= (int)vCollisionInfos.size() ) {
        vCollisionInfos.resize(nNumValidCollisions+1);
    }
    CollisionPairInfo& addcpinfo = vCollisionInfos[nNumValidCollisions];
    addcpinfo.Reset(); // might be old data
    addcpinfo.SetFirstCollision(link1.GetParent()->GetName(), link1.GetName(), std::string());
    // now check if there exists one like it already
    for(int icollision = 0; icollision < nNumValidCollisions; ++icollision) {
        const CollisionPairInfo& checkcpinfo = vCollisionInfos[icollision];
        if( checkcpinfo.bodyLinkGeom1Name == addcpinfo.bodyLinkGeom1Name && checkcpinfo.bodyLinkGeom2Name.empty() ) {
            return icollision;
        }
    }
    // return the new index
    return nNumValidCollisions++;
}

int CollisionReport::AddLinkCollision(const KinBody::Link& link1, const KinBody::Link& link2)
{
    // first write the collision as if it was unique
    if( nNumValidCollisions+1 >= (int)vCollisionInfos.size() ) {
        vCollisionInfos.resize(nNumValidCollisions+1);
    }
    CollisionPairInfo& addcpinfo = vCollisionInfos[nNumValidCollisions];
    addcpinfo.Reset(); // might be old data
    addcpinfo.SetFirstCollision(link1.GetParent()->GetName(), link1.GetName(), std::string());
    addcpinfo.SetSecondCollision(link2.GetParent()->GetName(), link2.GetName(), std::string());
    // now check if there exists one like it already
    for(int icollision = 0; icollision < nNumValidCollisions; ++icollision) {
        const CollisionPairInfo& checkcpinfo = vCollisionInfos[icollision];
        if( checkcpinfo.bodyLinkGeom1Name == addcpinfo.bodyLinkGeom1Name && checkcpinfo.bodyLinkGeom2Name == addcpinfo.bodyLinkGeom2Name ) {
            return icollision;
        }
    }
    // return the new index
    return nNumValidCollisions++;
}

int CollisionReport::AddLinkGeomCollision(const KinBody::LinkConstPtr& plink1, const std::string& geomname1, const KinBody::LinkConstPtr& plink2, const std::string& geomname2)
{
    // first write the collision as if it was unique
    if( nNumValidCollisions+1 >= (int)vCollisionInfos.size() ) {
        vCollisionInfos.resize(nNumValidCollisions+1);
    }
    CollisionPairInfo& addcpinfo = vCollisionInfos[nNumValidCollisions];
    addcpinfo.Reset(); // might be old data
    if( !!plink1 ) {
        addcpinfo.SetFirstCollision(plink1->GetParent()->GetName(), plink1->GetName(), geomname1);
    }
    if( !!plink2 ) {
        addcpinfo.SetSecondCollision(plink2->GetParent()->GetName(), plink2->GetName(), geomname2);
    }

    // now check if there exists one like it already
    for(int icollision = 0; icollision < nNumValidCollisions; ++icollision) {
        const CollisionPairInfo& checkcpinfo = vCollisionInfos[icollision];
        if( checkcpinfo.bodyLinkGeom1Name == addcpinfo.bodyLinkGeom1Name && checkcpinfo.bodyLinkGeom2Name == addcpinfo.bodyLinkGeom2Name ) {
            return icollision;
        }
    }
    // return the new index
    return nNumValidCollisions++;
}

int CollisionReport::SetLinkGeomCollision(const KinBody::LinkConstPtr& plink1, const std::string& geomname1, const KinBody::LinkConstPtr& plink2, const std::string& geomname2)
{
    if( vCollisionInfos.size() == 0 ) {
        vCollisionInfos.resize(1);
    }
    CollisionPairInfo& addcpinfo = vCollisionInfos[0];
    addcpinfo.Reset(); // might be old data
    if( !!plink1 ) {
        addcpinfo.SetFirstCollision(plink1->GetParent()->GetName(), plink1->GetName(), geomname1);
    }
    if( !!plink2 ) {
        addcpinfo.SetSecondCollision(plink2->GetParent()->GetName(), plink2->GetName(), geomname2);
    }
    nNumValidCollisions = 1;
    return 0;
}

CollisionOptionsStateSaver::CollisionOptionsStateSaver(CollisionCheckerBasePtr p, int newoptions, bool required)
{
    _oldoptions = p->GetCollisionOptions();
    _p = p;
    if( !_p->SetCollisionOptions(newoptions) ) {
        if( required ) {
            throw openrave_exception(str(boost::format(_("Failed to set collision options %d in checker %s\n"))%newoptions%_p->GetXMLId()));
        }
    }
}

CollisionOptionsStateSaver::~CollisionOptionsStateSaver()
{
    _p->SetCollisionOptions(_oldoptions);
}

CollisionOptionsStateSaverAll::CollisionOptionsStateSaverAll(const EnvironmentBasePtr pEnv, const int options, const bool required, const CollisionOptionsModificationType modificationType)
{
    // initialize checkers and old options
    pEnv->GetCollisionCheckers(_vCheckers);
    _vOldOptions.reserve(_vCheckers.size());
    for(const CollisionCheckerBasePtr& pChecker : _vCheckers) {
        _vOldOptions.push_back(pChecker->GetCollisionOptions());
    }

    // try setting
    int iCheckerFailed = -1;
    for(int iChecker = 0; iChecker < (int)_vCheckers.size(); ++iChecker) {
        const int newOptions = _ComputeNewOption(_vOldOptions.at(iChecker), options, modificationType);
        if( !_vCheckers.at(iChecker)->SetCollisionOptions(newOptions) && required ) {
            iCheckerFailed = iChecker;
            break;
        }
    }

    // if failed, restore first, and the throw.
    if( iCheckerFailed >= 0 ) {
        _Restore();
        throw openrave_exception(str(boost::format(_("Failed to set collision options %d in checker %s\n"))%(_vOldOptions[iCheckerFailed] | options)%_vCheckers[iCheckerFailed]->GetXMLId()));
    }
}

CollisionOptionsStateSaverAll::CollisionOptionsStateSaverAll(const EnvironmentBasePtr pEnv, const std::vector<int>& vOptions, const bool required)
{
    // initialize checkers and old options
    pEnv->GetCollisionCheckers(_vCheckers);
    OPENRAVE_ASSERT_OP(_vCheckers.size(), ==, vOptions.size());
    _vOldOptions.reserve(_vCheckers.size());
    for(const CollisionCheckerBasePtr& pChecker : _vCheckers) {
        _vOldOptions.push_back(pChecker->GetCollisionOptions());
    }

    // try setting
    int iCheckerFailed = -1;
    for(int iChecker = 0; iChecker < (int)_vCheckers.size(); ++iChecker) {
        if( !_vCheckers.at(iChecker)->SetCollisionOptions(vOptions[iChecker]) && required ) {
            iCheckerFailed = iChecker;
            break;
        }
    }

    // if failed, restore first, and the throw.
    if( iCheckerFailed >= 0 ) {
        _Restore();
        throw openrave_exception(str(boost::format(_("Failed to set collision options %d in checker %s\n"))%vOptions[iCheckerFailed]%_vCheckers[iCheckerFailed]->GetXMLId()));
    }
}

CollisionOptionsStateSaverAll::~CollisionOptionsStateSaverAll()
{
    _Restore();
}

void CollisionOptionsStateSaverAll::_Restore()
{
    for(int iChecker = 0; iChecker < (int)_vCheckers.size(); ++iChecker) {
        _vCheckers.at(iChecker)->SetCollisionOptions(_vOldOptions.at(iChecker));
    }
}

int CollisionOptionsStateSaverAll::_ComputeNewOption(const int oldOptions, const int optionsModification, const CollisionOptionsModificationType modificationType)
{
    switch( modificationType )
    {
    case COMT_Add:
        return oldOptions | optionsModification;
    case COMT_Remove:
        return oldOptions & (~optionsModification);
    case COMT_Set:
        return optionsModification;
    default:
        break;
    }
    throw openrave_exception(str(boost::format(_("Uknown CollisionOptionModificationType %s\n"))%modificationType));
}

} // end namespace OpenRAVE
