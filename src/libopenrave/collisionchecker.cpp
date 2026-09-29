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

static void _SetCollisionBodyLinkGeomName(std::string& bodyLinkGeomName, const std::string& bodyname, const std::string& linkname, const std::string& geomname)
{
    bodyLinkGeomName = bodyname;
    bodyLinkGeomName.push_back(' ');
    bodyLinkGeomName += linkname;
    bodyLinkGeomName.push_back(' ');
    bodyLinkGeomName += geomname;
}

void CollisionPairInfo::SetFirstCollision(const std::string& bodyname, const std::string& linkname, const std::string& geomname)
{
    _SetCollisionBodyLinkGeomName(bodyLinkGeom1Name, bodyname, linkname, geomname);
}

void CollisionPairInfo::SetSecondCollision(const std::string& bodyname, const std::string& linkname, const std::string& geomname)
{
    _SetCollisionBodyLinkGeomName(bodyLinkGeom2Name, bodyname, linkname, geomname);
}

inline void _ExtractBodyName(const std::string& bodyLinkGeomName, string_view& bodyname)
{
    bodyname = string_view();
    if( !bodyLinkGeomName.empty() ) {
        size_t index = bodyLinkGeomName.find_first_of(' ');
        OPENRAVE_ASSERT_FORMAT((index != std::string::npos), "Failed to extract body name since bodyLinkGeomName='%s' does not contain the first white space.", bodyLinkGeomName, ORE_Assert);
        bodyname = string_view(bodyLinkGeomName.c_str(), index);
    }
}

void CollisionPairInfo::ExtractFirstBodyName(string_view& bodyname) const
{
    _ExtractBodyName(bodyLinkGeom1Name, bodyname);
}

void CollisionPairInfo::ExtractSecondBodyName(string_view& bodyname) const
{
    _ExtractBodyName(bodyLinkGeom2Name, bodyname);
}

inline void _ExtractLinkName(const std::string& bodyLinkGeomName, string_view& linkname)
{
    linkname = string_view();
    if( bodyLinkGeomName.empty() ) {
        return;
    }
    size_t firstindex = bodyLinkGeomName.find_first_of(' ');
    OPENRAVE_ASSERT_FORMAT((firstindex != std::string::npos), "Failed to extract link name since bodyLinkGeomName='%s' does not contain the first white space.", bodyLinkGeomName, ORE_Assert);

    size_t secondindex = bodyLinkGeomName.find_first_of(' ', firstindex+1);
    OPENRAVE_ASSERT_FORMAT((secondindex != std::string::npos), "Failed to extract link name since bodyLinkGeomName='%s' does not contain the second white space.", bodyLinkGeomName, ORE_Assert);
    linkname = string_view(bodyLinkGeomName.c_str()+firstindex+1,secondindex - firstindex - 1);
}

void CollisionPairInfo::ExtractFirstLinkName(string_view& linkname) const
{
    _ExtractLinkName(bodyLinkGeom1Name, linkname);
}

void CollisionPairInfo::ExtractSecondLinkName(string_view& linkname) const
{
    _ExtractLinkName(bodyLinkGeom2Name, linkname);
}

inline void _ExtractBodyLinkNames(const std::string& bodyLinkGeomName, string_view& bodyname, string_view& linkname)
{
    bodyname = string_view();
    linkname = string_view();
    if( bodyLinkGeomName.empty() ) {
        return;
    }
    size_t firstindex = bodyLinkGeomName.find_first_of(' ');
    OPENRAVE_ASSERT_FORMAT((firstindex != std::string::npos), "Failed to extract body/link name since bodyLinkGeomName='%s' does not contain the first white space.", bodyLinkGeomName, ORE_Assert);

    bodyname = string_view(bodyLinkGeomName.c_str(),firstindex);

    size_t secondindex = bodyLinkGeomName.find_first_of(' ', firstindex+1);
    OPENRAVE_ASSERT_FORMAT((secondindex != std::string::npos), "Failed to extract body/link name since bodyLinkGeomName='%s' does not contain the second white space.", bodyLinkGeomName, ORE_Assert);
    linkname = string_view(bodyLinkGeomName.c_str()+firstindex+1, secondindex - firstindex - 1);
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
    if( bodyLinkGeomName.empty() ) {
        return;
    }

    size_t firstindex = bodyLinkGeomName.find_first_of(' ');
    OPENRAVE_ASSERT_FORMAT((firstindex != std::string::npos), "Failed to extract body/link/geom name since bodyLinkGeomName='%s' does not contain the first white space.", bodyLinkGeomName, ORE_Assert);

    bodyname = string_view(bodyLinkGeomName.c_str(),firstindex);

    size_t secondindex = bodyLinkGeomName.find_first_of(' ', firstindex+1);
    OPENRAVE_ASSERT_FORMAT((secondindex != std::string::npos), "Failed to extract body/link/geom name since bodyLinkGeomName='%s' does not contain the second white space.", bodyLinkGeomName, ORE_Assert);

    linkname = string_view(bodyLinkGeomName.c_str()+firstindex+1, secondindex - firstindex - 1);
    geomname = string_view(bodyLinkGeomName.c_str()+secondindex+1, bodyLinkGeomName.size()-secondindex-1);
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
    if( bodyLinkGeomName.empty() ) {
        return -1; // no link
    }

    // compare the linkname first since getting parent requires atomic operations
    size_t firstindex = bodyLinkGeomName.find_first_of(' ');
    OPENRAVE_ASSERT_FORMAT((firstindex != std::string::npos), "Failed to compare link since bodyLinkGeomName='%s' does not contain the first white space.", bodyLinkGeomName, ORE_Assert);

    string_view linkname;

    size_t secondindex = bodyLinkGeomName.find_first_of(' ', firstindex+1);
    OPENRAVE_ASSERT_FORMAT((secondindex != std::string::npos), "Failed to compare link since bodyLinkGeomName='%s' does not contain the second white space.", bodyLinkGeomName, ORE_Assert);
    linkname = string_view(bodyLinkGeomName.c_str()+firstindex+1, secondindex - firstindex - 1);

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

    if( bodyLinkGeomName.empty() ) {
        return -1;
    }

    const size_t firstindex = bodyLinkGeomName.find_first_of(' ');
    OPENRAVE_ASSERT_FORMAT((firstindex != std::string::npos), "Failed to find matching link index since bodyLinkGeomName='%s' does not contain the second white space.", bodyLinkGeomName, ORE_Assert);

    const std::string& bodyname = vlinks[0]->GetParent()->GetName();
    if( bodyname.size() != firstindex ) {
        return -1;
    }

    if( strncmp(bodyname.c_str(), bodyLinkGeomName.c_str(), firstindex) != 0 ) {
        return -1;
    }

    // body matches, now check the links
    const size_t linkNameStartIndex = firstindex+1;
    const size_t secondindex = bodyLinkGeomName.find_first_of(' ', linkNameStartIndex);
    OPENRAVE_ASSERT_FORMAT((secondindex != std::string::npos), "Failed to find matching link index since bodyLinkGeomName='%s' does not contain the second white space.", bodyLinkGeomName, ORE_Assert);
    size_t nLinkNameLength = secondindex - linkNameStartIndex;

    for(int ilink = 0; ilink < (int)vlinks.size(); ++ilink) {
        const KinBody::LinkPtr& plink = vlinks[ilink];
        if( plink->GetName().size() == nLinkNameLength ) {
            if( strncmp(plink->GetName().c_str(), bodyLinkGeomName.c_str()+linkNameStartIndex, nLinkNameLength) == 0 ) {
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
    return _FindMatchingLinkIndex(bodyLinkGeom2Name, vlinks);
}

inline int _CompareLinkName(const std::string& bodyLinkGeomName, const std::string& linkname)
{
    if( bodyLinkGeomName.empty() ) {
        return -1;
    }

    size_t index = bodyLinkGeomName.find_first_of(' ');
    OPENRAVE_ASSERT_FORMAT((index != std::string::npos), "Failed to compare link name since bodyLinkGeomName='%s' does not contain the first white space.", bodyLinkGeomName, ORE_Assert);

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

    size_t nNameLength = bodyLinkGeomName.find_first_of(' ');
    OPENRAVE_ASSERT_FORMAT((nNameLength != std::string::npos), "Failed to extract body since bodyLinkGeomName='%s' does not contain the first white space.", bodyLinkGeomName, ORE_Assert);
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

} // end namespace OpenRAVE
