// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov (rosen.diankov@gmail.com)
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

SimpleSensorSystem::SimpleXMLReader::SimpleXMLReader(boost::shared_ptr<XMLData> p) : _pdata(p)
{
}

BaseXMLReader::ProcessElement SimpleSensorSystem::SimpleXMLReader::startElement(const std::string& name, const AttributesList& atts)
{
    ss.str("");
    if((name != _pdata->GetXMLId())&&(name != "offsetlink")&&(name != "id")&&(name != "sid")&&(name != "translation")&&(name != "rotationmat")&&(name != "rotationaxis")&&(name != "quat")&&(name != "pretranslation")&&(name != "prerotation")&&(name != "prerotationaxis")&&(name != "prequat")) {
        return PE_Pass;
    }
    return PE_Support;
}

bool SimpleSensorSystem::SimpleXMLReader::endElement(const std::string& name)
{
    if( name == "offsetlink" ) {
        ss >> _pdata->strOffsetLink;
    }
    else if( name == "id" ) {
        ss >> _pdata->id;
    }
    else if( name == "sid" ) {
        ss >> _pdata->sid;
    }
    else if( name == "translation" ) {
        ss >> _pdata->transOffset.trans.x >> _pdata->transOffset.trans.y >> _pdata->transOffset.trans.z;
    }
    else if( name == "rotationmat" ) {
        TransformMatrix m;
        ss >> m.m[0] >> m.m[1] >> m.m[2] >> m.m[4] >> m.m[5] >> m.m[6] >> m.m[8] >> m.m[9] >> m.m[10];
        _pdata->transOffset.rot = Transform(m).rot;
    }
    else if( name == "rotationaxis" ) {
        Vector axis; dReal fang;
        ss >> axis.x >> axis.y >> axis.z >> fang;
        _pdata->transOffset.rot = quatFromAxisAngle(axis,fang*dReal(PI/180.0));
    }
    else if( name == "quat" ) {
        ss >> _pdata->transOffset.rot;
    }
    else if( name == "pretranslation") {
        ss >> _pdata->transPreOffset.trans.x >> _pdata->transPreOffset.trans.y >> _pdata->transPreOffset.trans.z;
    }
    else if( name == "prerotationmat") {
        TransformMatrix m;
        ss >> m.m[0] >> m.m[1] >> m.m[2] >> m.m[4] >> m.m[5] >> m.m[6] >> m.m[8] >> m.m[9] >> m.m[10];
        _pdata->transPreOffset.rot = Transform(m).rot;
    }
    else if( name == "prerotationaxis") {
        Vector axis; dReal fang;
        ss >> axis.x >> axis.y >> axis.z >> fang;
        _pdata->transPreOffset.rot = quatFromAxisAngle(axis,fang*dReal(PI/180.0));
    }
    else if( name == "prequat") {
        ss >> _pdata->transPreOffset.rot;
    }
    else if( name == tolowerstring(_pdata->GetXMLId()) ) {
        return true;
    }
    if( !ss ) {
        RAVELOG_WARN(str(boost::format("error parsing %s\n")%name));
    }
    return false;
}

void SimpleSensorSystem::SimpleXMLReader::characters(const std::string& ch)
{
    ss.clear();
    ss << ch;
}

BaseXMLReaderPtr SimpleSensorSystem::CreateXMLReaderId(const string& xmlid, InterfaceBasePtr ptr, const AttributesList& atts)
{
    return BaseXMLReaderPtr(new SimpleXMLReader(boost::shared_ptr<XMLData>(new XMLData(xmlid))));
}

UserDataPtr SimpleSensorSystem::RegisterXMLReaderId(EnvironmentBasePtr penv, const string& xmlid)
{
    return RaveRegisterXMLReader(PT_KinBody,xmlid, boost::bind(&SimpleSensorSystem::CreateXMLReaderId,xmlid, _1,_2));
}

SimpleSensorSystem::SimpleSensorSystem(const std::string& xmlid, EnvironmentBasePtr penv) : SensorSystemBase(penv), _expirationtime(2000000), _bShutdown(false), _threadUpdate(boost::bind(&SimpleSensorSystem::_UpdateBodiesThread,this))
{
    _xmlid = xmlid;
    std::transform(_xmlid.begin(), _xmlid.end(), _xmlid.begin(), ::tolower);
}

SimpleSensorSystem::~SimpleSensorSystem()
{
    Reset();
    _bShutdown = true;
    _threadUpdate.join();
}

void SimpleSensorSystem::Reset()
{
    boost::mutex::scoped_lock lock(_mutex);
    _mapbodies.clear();
}

void SimpleSensorSystem::AddRegisteredBodies(const std::vector<KinBodyPtr>& vbodies)
{
    // go through all bodies in the environment and check for mocap data
    FOREACHC(itbody, vbodies) {
        boost::shared_ptr<XMLData> pmocapdata = boost::dynamic_pointer_cast<XMLData>((*itbody)->GetReadableInterface(_xmlid));
        if( !!pmocapdata ) {
            KinBody::ManageDataPtr p = AddKinBody(*itbody, pmocapdata);
            if( !!p ) {
                p->Lock(true);
            }
        }
    }
}

KinBody::ManageDataPtr SimpleSensorSystem::AddKinBody(KinBodyPtr pbody, XMLReadableConstPtr _pdata)
{
    BOOST_ASSERT(pbody->GetEnv()==GetEnv());
    boost::shared_ptr<XMLData const> pdata = boost::static_pointer_cast<XMLData const>(_pdata);
    if( !pdata ) {
        pdata = boost::dynamic_pointer_cast<XMLData const>(pbody->GetReadableInterface(_xmlid));
        if( !pdata ) {
            RAVELOG_VERBOSE(str(boost::format("failed to find manage data for body %s\n")%pbody->GetName()));
            return KinBody::ManageDataPtr();
        }
    }

    boost::mutex::scoped_lock lock(_mutex);
    if( _mapbodies.find(pbody->GetEnvironmentId()) != _mapbodies.end() ) {
        RAVELOG_WARN(str(boost::format("body %s already added\n")%pbody->GetName()));
        return KinBody::ManageDataPtr();
    }

    boost::shared_ptr<BodyData> b = CreateBodyData(pbody, pdata);
    b->lastupdated = GetMicroTime();
    _mapbodies[pbody->GetEnvironmentId()] = b;
    RAVELOG_VERBOSE(str(boost::format("system adding body %s (%s), total: %d\n")%pbody->GetName()%pbody->GetURI()%_mapbodies.size()));
    SetManageData(pbody,b);
    return b;
}

bool SimpleSensorSystem::RemoveKinBody(KinBodyPtr pbody)
{
    boost::mutex::scoped_lock lock(_mutex);
    bool bSuccess = _mapbodies.erase(pbody->GetEnvironmentId())>0;
    RAVELOG_VERBOSE(str(boost::format("system removing body %s %s\n")%pbody->GetName()%(bSuccess ? "succeeded" : "failed")));
    return bSuccess;
}

bool SimpleSensorSystem::IsBodyPresent(KinBodyPtr pbody)
{
    boost::mutex::scoped_lock lock(_mutex);
    return _mapbodies.find(pbody->GetEnvironmentId()) != _mapbodies.end();
}

bool SimpleSensorSystem::EnableBody(KinBodyPtr pbody, bool bEnable)
{
    boost::mutex::scoped_lock lock(_mutex);
    BODIES::iterator it = _mapbodies.find(pbody->GetEnvironmentId());
    if( it == _mapbodies.end() ) {
        RAVELOG_WARN("trying to %s body %s that is not in system\n", bEnable ? "enable" : "disable", pbody->GetName().c_str());
        return false;
    }

    it->second->bEnabled = bEnable;
    return true;
}

bool SimpleSensorSystem::SwitchBody(KinBodyPtr pbody1, KinBodyPtr pbody2)
{
    //boost::mutex::scoped_lock lock(_mutex);
    BODIES::iterator it = _mapbodies.find(pbody1->GetEnvironmentId());
    boost::shared_ptr<BodyData> pb1,pb2;
    if( it != _mapbodies.end() ) {
        pb1 = it->second;
    }
    it = _mapbodies.find(pbody2->GetEnvironmentId());
    if( it != _mapbodies.end() ) {
        pb2 = it->second;
    }
    if( !pb1 || !pb2 ) {
        return false;
    }
    if( !!pb1 ) {
        pb1->SetBody(pbody2);
    }
    if( !!pb2 ) {
        pb2->SetBody(pbody1);
    }
    return true;
}

boost::shared_ptr<SimpleSensorSystem::BodyData> SimpleSensorSystem::CreateBodyData(KinBodyPtr pbody, boost::shared_ptr<XMLData const> pdata)
{
    boost::shared_ptr<XMLData> pnewdata(new XMLData(_xmlid));
    pnewdata->copy(pdata);
    return boost::shared_ptr<BodyData>(new BodyData(RaveInterfaceCast<SimpleSensorSystem>(shared_from_this()),pbody, pnewdata));
}

void SimpleSensorSystem::_UpdateBodies(list<SimpleSensorSystem::SNAPSHOT>& listbodies)
{
    EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex()); // always lock environment to preserve mutex order
    uint64_t curtime = GetMicroTime();
    if( listbodies.size() > 0 ) {

        FOREACH(it, listbodies) {
            BOOST_ASSERT( it->first->IsEnabled() );

            KinBody::LinkPtr plink = it->first->GetOffsetLink();
            if( !plink ) {
                continue;
            }
            // transform with respect to offset link
            TransformMatrix tlink = plink->GetTransform();
            TransformMatrix tbase = plink->GetParent()->GetTransform();
            TransformMatrix toffset = tbase * tlink.inverse() * it->first->_initdata->transOffset;
            TransformMatrix tfinal = toffset * it->second*it->first->_initdata->transPreOffset;

            plink->GetParent()->SetTransform(tfinal);
            it->first->lastupdated = curtime;
            it->first->tnew = it->second;

            if( !it->first->IsPresent() ) {
                RAVELOG_VERBOSE(str(boost::format("updating body %s\n")%plink->GetParent()->GetName()));
            }
            it->first->bPresent = true;
        }
    }

    boost::mutex::scoped_lock lock(_mutex);
    BODIES::iterator itbody = _mapbodies.begin();
    while(itbody != _mapbodies.end()) {
        KinBody::LinkPtr plink = itbody->second->GetOffsetLink();
        if( !!plink &&(plink->GetParent()->GetEnvironmentId()==0)) {
            _mapbodies.erase(itbody++);
            continue;
        }
        else if( curtime-itbody->second->lastupdated > _expirationtime ) {
            if( !itbody->second->IsLocked() ) {
                if( !!plink ) {
                    //RAVELOG_VERBOSE(str(boost::format("object %s expired %fs\n")%plink->GetParent()->GetName()*((curtime-itbody->second->lastupdated)*1e-6f)));
                    GetEnv()->Remove(plink->GetParent());
                }
                _mapbodies.erase(itbody++);
                continue;
            }

            if( itbody->second->IsPresent() && !!plink ) {
                RAVELOG_VERBOSE(str(boost::format("body %s not present\n")%plink->GetParent()->GetName()));
            }
            itbody->second->bPresent = false;
        }

        ++itbody;
    }
}

void SimpleSensorSystem::_UpdateBodiesThread()
{
    list< SNAPSHOT > listbodies;

    while(!_bShutdown) {
        {
            _UpdateBodies(listbodies);
        }
        Sleep(10); // 10ms
    }
}

}
