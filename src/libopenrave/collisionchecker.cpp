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

CollisionReport::CollisionReport() {
    vCollisionInfos.resize(1);
}

void CollisionReport::CONTACT::SaveToJson(rapidjson::Value& rContact, rapidjson::Document::AllocatorType& alloc) const
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

void CollisionReport::CollisionPairInfo::SetFirstCollision(const std::string& bodyname, const std::string& linkname, const std::string& geomname)
{
    bodyLinkGeom1Name = bodyname;
    bodyLinkGeom1Name.push_back(' ');
    bodyLinkGeom1Name += linkname;
    bodyLinkGeom1Name.push_back(' ');
    bodyLinkGeom1Name += geomname;
}

void CollisionReport::CollisionPairInfo::SetSecondCollision(const std::string& bodyname, const std::string& linkname, const std::string& geomname)
{
    bodyLinkGeom2Name = bodyname;
    bodyLinkGeom2Name.push_back(' ');
    bodyLinkGeom2Name += linkname;
    bodyLinkGeom2Name.push_back(' ');
    bodyLinkGeom2Name += geomname;
}

void CollisionReport::CollisionPairInfo::SaveToJson(rapidjson::Value& rCollision, rapidjson::Document::AllocatorType& alloc) const
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

void CollisionReport::Reset(int coloptions)
{
    options = coloptions;
    if( !(nKeepPrevious & 1) ) {
        minDistance = 1e20f;
        numWithinTol = 0;
        nNumValidCollisions = 0;
    }
}

CollisionReport& CollisionReport::operator=(const CollisionReport& rhs)
{
    nNumValidCollisions = rhs.nNumValidCollisions;
    if( vCollisionInfos.size() < nNumValidCollisions ) {
        vCollisionInfos.resize(nNumValidCollisions);
    }
    OPENRAVE_ASSERT_OP(rhs.nNumValidCollisions,<=,rhs.vCollisionInfos.size()); // sanity check
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
    stringstream s;
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
    if( minDistance < 1e10 ) {
        orjson::SetJsonValueByKey(rCollisionReport, "minDistance", minDistance, alloc);
    }
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
