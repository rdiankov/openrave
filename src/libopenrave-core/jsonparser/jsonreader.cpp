// -*- coding: utf-8 -*-
// Copyright (C) 2013 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "jsoncommon.h"

#include <openrave/openravejson.h>
#include <rapidjson/istreamwrapper.h>
#include <string>
#include <fstream>

namespace OpenRAVE {

class JSONReader {
public:

    JSONReader(const AttributesList& atts, EnvironmentBasePtr penv): _penv(penv), _prefix("")
    {
        FOREACHC(itatt, atts) {
            if (itatt->first == "prefix") {
                _prefix = itatt->second;
            }
            else if (itatt->first == "openravescheme")
            {
                std::stringstream ss(itatt->second);
                _vOpenRAVESchemeAliases = std::vector<std::string>((istream_iterator<std::string>(ss)), istream_iterator<std::string>());
            }
        }
        if (_vOpenRAVESchemeAliases.size() == 0) {
            _vOpenRAVESchemeAliases.push_back("openrave");
        }

        // set global scale when initalize jsonreader.
        _fGlobalScale = 1.0 / _penv->GetUnit().second;
    }

    virtual ~JSONReader()
    {
    }

    bool InitFromFile(const std::string& filename)
    {
        _filename = filename;
        _doc = _OpenDocument(_filename);
        return true;
    }

    bool InitFromURI(const std::string& uri)
    {
        _uri = uri;
        _filename = _ResolveURI(_uri);
        _doc = _OpenDocument(_filename);
        return true;
    }

    bool InitFromData(const std::string& data)
    {
        _doc = _ParseDocument(data);
        return true;
    }

    bool InitFromDocument(const rapidjson::Document& doc)
    {
        boost::shared_ptr<rapidjson::Document> pDoc;
        pDoc.reset(new rapidjson::Document);
        pDoc->CopyFrom(doc, pDoc->GetAllocator());
        _docs[""] = pDoc;
        _doc = pDoc;
        return true;
    }

    bool ExtractAll() {
        bool allSucceeded = true;
        // extra all bodies and add to env
        std::pair<std::string, dReal> unit;
        openravejson::LoadJsonValueByKey(*_doc, "unit", unit);
        _penv->SetUnit(unit);

        dReal fUnitScale = _GetUnitScale();
        if (_doc->HasMember("bodies") && (*_doc)["bodies"].IsArray()) {
            std::map<KinBodyPtr, std::vector<KinBody::GrabbedInfoConstPtr>> mapKinBodyGrabbedInfos;
            for (rapidjson::Value::ValueIterator itr = (*_doc)["bodies"].Begin(); itr != (*_doc)["bodies"].End(); ++itr) {
                KinBodyPtr pbody;
                if (_Extract(*itr, pbody)) {
                    _penv->Add(pbody, true);

                    // set dof values
                    if (itr->HasMember("dofValues")) {
                        std::vector<dReal> vDOFValues;
                        openravejson::LoadJsonValueByKey(*itr, "dofValues", vDOFValues);
                        pbody->SetDOFValues(vDOFValues, KinBody::CLA_Nothing);
                    }

                    // parse grabbed infos
                    if (itr->HasMember("grabbed") && (*itr)["grabbed"].IsArray()) {
                        for(rapidjson::Value::ValueIterator itGrabInfo = (*itr)["grabbed"].Begin(); itGrabInfo != (*itr)["grabbed"].End(); ++itGrabInfo){
                            KinBody::GrabbedInfoPtr pGrabbedInfo(new KinBody::GrabbedInfo());
                            pGrabbedInfo->DeserializeJSON(*itGrabInfo, fUnitScale);
                            mapKinBodyGrabbedInfos[pbody].push_back(pGrabbedInfo);
                        }
                    }
                } else {
                    allSucceeded = false;
                }
            }
            if (allSucceeded) {
                // reset grabbed
                FOREACH(it, mapKinBodyGrabbedInfos) {
                    it->first->ResetGrabbed(it->second);
                }
            }
            return allSucceeded;
        }
        return false;
    }

    bool ExtractFirst(KinBodyPtr& ppbody) {
        // extract the first articulated system found.
        if (_doc->HasMember("bodies") && (*_doc)["bodies"].IsArray()) {
            for (rapidjson::Value::ValueIterator itr = (*_doc)["bodies"].Begin(); itr != (*_doc)["bodies"].End(); ++itr) {
                return _Extract(*itr, ppbody);
            }
        }
        return false;
    }

    bool ExtractFirst(RobotBasePtr& pprobot) {
        // extract the first robot 
        if (_doc->HasMember("bodies") && (*_doc)["bodies"].IsArray()) {
            for (rapidjson::Value::ValueIterator itr = (*_doc)["bodies"].Begin(); itr != (*_doc)["bodies"].End(); ++itr) {
                return _Extract(*itr, pprobot);
            }
        }
        return false;
    }

    bool ExtractOne(KinBodyPtr& ppbody, const string& uri) {
        std::string scheme, path, fragment;
        _ParseURI(uri, scheme, path, fragment);            
        if (fragment == "") {
            return ExtractFirst(ppbody);
        }

        // find the body by uri
        if (_doc->HasMember("bodies") && (*_doc)["bodies"].IsArray()) {
            for (rapidjson::Value::ValueIterator itr = (*_doc)["bodies"].Begin(); itr != (*_doc)["bodies"].End(); ++itr) {
                std::string bodyUri;
                openravejson::LoadJsonValueByKey(*itr, "uri", bodyUri);
                if (bodyUri == std::string("#") + fragment) {
                    return _Extract(*itr, ppbody);
                }
            }
        }
        return false;
    }

    bool ExtractOne(RobotBasePtr& pprobot, const string& uri) {
        std::string scheme, path, fragment;
        _ParseURI(uri, scheme, path, fragment);
        if (fragment == "") {
            return ExtractFirst(pprobot);
        }

        // find the body by uri
        if (_doc->HasMember("bodies") && (*_doc)["bodies"].IsArray()) {
            for (rapidjson::Value::ValueIterator itr = (*_doc)["bodies"].Begin(); itr != (*_doc)["bodies"].End(); ++itr) {
                std::string bodyUri;
                openravejson::LoadJsonValueByKey(*itr, "uri", bodyUri);
                if (bodyUri == std::string("#") + fragment) {
                    return _Extract(*itr, pprobot);
                }
            }
        }
        return false;
    }

protected:
    inline dReal _GetUnitScale()
    {
        std::pair<std::string, dReal> unit;
        openravejson::LoadJsonValueByKey(*_doc, "unit", unit);
        if (unit.first == "mm")
        {
            unit.second *= 0.001;
        }
        else if (unit.first == "cm")
        {
            unit.second *= 0.01;
        }
        dReal scale = unit.second / _fGlobalScale;
        return scale;
    }

    std::string _CanonicalizeURI(const std::string& uri)
    {
        std::string scheme, path, fragment;
        _ParseURI(uri, scheme, path, fragment);

        if (scheme == "" && path == "") {
            if (_uri != "") {
                std::string scheme2, path2, fragment2;
                _ParseURI(_uri, scheme2, path2, fragment2);
                return scheme2 + ":" + path2 + "#" + fragment;
            }
            return std::string("file:") + _filename + "#" + fragment;
        }
        return uri;
    }

    bool _Extract(const rapidjson::Value &bodyValue, KinBodyPtr& pbody)
    {
        std::string uri;
        openravejson::LoadJsonValueByKey(bodyValue, "uri", uri);

        rapidjson::Value::ValueIterator object = _ResolveObject(uri);

        if (openravejson::GetJsonValueByKey<bool>(*object, "isRobot")) {
            RobotBasePtr probot;
            if (_Extract(bodyValue, probot)) {
                pbody = probot;
                return true;
            }
            return false;
        }

        dReal fUnitScale = _GetUnitScale();

        std::vector<KinBody::LinkInfoPtr> vLinkInfos;
        _ExtractLinks(*object, vLinkInfos, fUnitScale);
        
        std::vector<KinBody::JointInfoPtr> vJointInfos;
        _ExtractJoints(*object, vJointInfos, fUnitScale);
        
        std::vector<KinBody::LinkInfoConstPtr> vLinkInfosConst(vLinkInfos.begin(), vLinkInfos.end());
        std::vector<KinBody::JointInfoConstPtr> vJointInfosConst(vJointInfos.begin(), vJointInfos.end());
        
        KinBodyPtr body = RaveCreateKinBody(_penv, "");
        if (!body->Init(vLinkInfosConst, vJointInfosConst, _CanonicalizeURI(uri))) {
            return false;
        }
        
        _ExtractReadableInterfaces(*object, body, fUnitScale);

        body->SetName(openravejson::GetJsonValueByKey<std::string>(bodyValue, "name"));

        if (bodyValue.HasMember("transform")) {
            Transform transform;
            openravejson::LoadJsonValueByKey(bodyValue, "transform", transform);
            body->SetTransform(transform);
        }

        pbody = body;
        return true;
    }

    bool _Extract(const rapidjson::Value &bodyValue, RobotBasePtr& probot)
    {
        std::string uri;
        openravejson::LoadJsonValueByKey(bodyValue, "uri", uri);

        rapidjson::Value::ValueIterator object = _ResolveObject(uri);

        if (!openravejson::GetJsonValueByKey<bool>(*object, "isRobot")) {
            return false;
        }

        dReal fUnitScale = _GetUnitScale();

        std::vector<KinBody::LinkInfoPtr> vLinkInfos;
        _ExtractLinks(*object, vLinkInfos, fUnitScale);
        
        std::vector<KinBody::JointInfoPtr> vJointInfos;
        _ExtractJoints(*object, vJointInfos, fUnitScale);

        std::vector<RobotBase::ManipulatorInfoPtr> vManipulatorInfos;
        _ExtractManipulators(*object, vManipulatorInfos, fUnitScale);

        std::vector<RobotBase::AttachedSensorInfoPtr> vAttachedSensorInfos;
        _ExtractAttachedSensors(*object, vAttachedSensorInfos, fUnitScale);

        std::vector<RobotBase::ConnectedBodyInfoPtr> vConnectedBodyInfos;
        _ExtractConnectedBodies(*object, vConnectedBodyInfos, fUnitScale);

        
        std::vector<KinBody::LinkInfoConstPtr> vLinkInfosConst(vLinkInfos.begin(), vLinkInfos.end());
        std::vector<KinBody::JointInfoConstPtr> vJointInfosConst(vJointInfos.begin(), vJointInfos.end());
        std::vector<RobotBase::ManipulatorInfoConstPtr> vManipulatorInfosConst(vManipulatorInfos.begin(), vManipulatorInfos.end());
        std::vector<RobotBase::AttachedSensorInfoConstPtr> vAttachedSensorInfosConst(vAttachedSensorInfos.begin(), vAttachedSensorInfos.end());
        std::vector<RobotBase::ConnectedBodyInfoConstPtr> vConnectedBodyInfosConst(vConnectedBodyInfos.begin(), vConnectedBodyInfos.end());
        
        RobotBasePtr robot = RaveCreateRobot(_penv, "");
        if (!robot->Init(vLinkInfosConst, vJointInfosConst, vManipulatorInfosConst, vAttachedSensorInfosConst, vConnectedBodyInfosConst, _CanonicalizeURI(uri))) {
            return false;
        }

        _ExtractReadableInterfaces(*object, robot, fUnitScale);

        robot->SetName(openravejson::GetJsonValueByKey<std::string>(bodyValue, "name"));

        if (bodyValue.HasMember("transform")) {
            Transform transform;
            openravejson::LoadJsonValueByKey(bodyValue, "transform", transform);
            robot->SetTransform(transform);
        }

        probot = robot;
        return true;
    }

    void _ExtractLinks(const rapidjson::Value &objectValue, std::vector<KinBody::LinkInfoPtr> &linkinfos, dReal fUnitScale)
    {
        if (objectValue.HasMember("links") && objectValue["links"].IsArray()) {
            linkinfos.reserve(linkinfos.size() + objectValue["links"].Size());
            for (rapidjson::Value::ConstValueIterator itr = objectValue["links"].Begin(); itr != objectValue["links"].End(); ++itr) {
                KinBody::LinkInfoPtr linkinfo(new KinBody::LinkInfo());
                linkinfo->DeserializeJSON(*itr, fUnitScale);
                linkinfos.push_back(linkinfo);
            }
        }
    }

    void _ExtractJoints(const rapidjson::Value &objectValue, std::vector<KinBody::JointInfoPtr> &jointinfos, dReal fUnitScale)
    {
        if (objectValue.HasMember("joints") && objectValue["joints"].IsArray()) {
            jointinfos.reserve(jointinfos.size() + objectValue["joints"].Size());
            for (rapidjson::Value::ConstValueIterator itr = objectValue["joints"].Begin(); itr != objectValue["joints"].End(); ++itr) {
                KinBody::JointInfoPtr jointinfo(new KinBody::JointInfo());
                jointinfo->DeserializeJSON(*itr, fUnitScale);
                jointinfos.push_back(jointinfo);
            }
        }
    }

    void _ExtractManipulators(const rapidjson::Value &objectValue, std::vector<RobotBase::ManipulatorInfoPtr> &manipinfos, dReal fUnitScale)
    {
        if (objectValue.HasMember("manipulators") && objectValue["manipulators"].IsArray()) {
            manipinfos.reserve(manipinfos.size() + objectValue["manipulators"].Size());
            for (rapidjson::Value::ConstValueIterator itr = objectValue["manipulators"].Begin(); itr != objectValue["manipulators"].End(); ++itr) {
                RobotBase::ManipulatorInfoPtr manipinfo(new RobotBase::ManipulatorInfo());
                manipinfo->DeserializeJSON(*itr, fUnitScale);
                manipinfos.push_back(manipinfo);
            }
        }
    }

    void _ExtractAttachedSensors(const rapidjson::Value &objectValue, std::vector<RobotBase::AttachedSensorInfoPtr> &attachedsensorinfos, dReal fUnitScale)
    {
        if (objectValue.HasMember("attachedSensors") && objectValue["attachedSensors"].IsArray()) {
            attachedsensorinfos.reserve(attachedsensorinfos.size() + objectValue["attachedSensors"].Size());
            for (rapidjson::Value::ConstValueIterator itr = objectValue["attachedSensors"].Begin(); itr != objectValue["attachedSensors"].End(); ++itr) {
                RobotBase::AttachedSensorInfoPtr attachedsensorinfo(new RobotBase::AttachedSensorInfo());
                attachedsensorinfo->DeserializeJSON(*itr, fUnitScale);
                attachedsensorinfos.push_back(attachedsensorinfo);
            }
        }
    }

    void _ExtractConnectedBodies(const rapidjson::Value &objectValue, std::vector<RobotBase::ConnectedBodyInfoPtr> &connectedbodyinfos, dReal fUnitScale)
    {
        if (objectValue.HasMember("connectedBodies") && objectValue["connectedBodies"].IsArray()) {
            for (rapidjson::Value::ConstValueIterator itr = objectValue["connectedBodies"].Begin(); itr != objectValue["connectedBodies"].End(); ++itr) {
                RobotBase::ConnectedBodyInfoPtr connectedbodyinfo(new RobotBase::ConnectedBodyInfo());
                connectedbodyinfo->DeserializeJSON(*itr, fUnitScale);

                rapidjson::Value::ValueIterator connectedBodyObject = _ResolveObject(connectedbodyinfo->_uri);
                _ExtractLinks(*connectedBodyObject, connectedbodyinfo->_vLinkInfos, fUnitScale);
                _ExtractJoints(*connectedBodyObject, connectedbodyinfo->_vJointInfos, fUnitScale);
                _ExtractManipulators(*connectedBodyObject, connectedbodyinfo->_vManipulatorInfos, fUnitScale);
                _ExtractAttachedSensors(*connectedBodyObject, connectedbodyinfo->_vAttachedSensorInfos, fUnitScale);
                connectedbodyinfos.push_back(connectedbodyinfo);
            }
        }
    }

    void _ExtractReadableInterfaces(const rapidjson::Value &objectValue, InterfaceBasePtr pInterface, dReal fUnitScale)
    {
        if (objectValue.HasMember("readableInterfaces") && objectValue["readableInterfaces"].IsObject()) {
            for (rapidjson::Value::ConstMemberIterator itr = objectValue["readableInterfaces"].MemberBegin(); itr != objectValue["readableInterfaces"].MemberEnd(); itr++) {
                std::string id = itr->name.GetString();
                BaseJSONReaderPtr pReader = RaveCallJSONReader(pInterface->GetInterfaceType(), id, pInterface, AttributesList());
                pReader->DeserializeJSON(itr->value, fUnitScale);
                JSONReadablePtr pReadable = pReader->GetReadable();
                if (!!pReadable) {
                    pInterface->SetReadableInterface(id, pReadable);
                }
            }
        }
    }

    /// \brief get the scheme of the uri, e.g. file: or openrave:
    void _ParseURI(const std::string& uri, std::string& scheme, std::string& path, std::string& fragment)
    {
        path = uri;
        size_t hashindex = path.find_last_of('#');
        if (hashindex != std::string::npos) {
            fragment = path.substr(hashindex + 1);
            path = path.substr(0, hashindex);
        }

        size_t colonindex = path.find_first_of(':');
        if (colonindex != std::string::npos) {
            // notice: in python code, like realtimerobottask3.py, it pass scheme as {openravescene: mujin}. No colon,
            scheme = path.substr(0, colonindex);
            path = path.substr(colonindex + 1);
        }

    }

    rapidjson::Value::ValueIterator _ResolveObject(const std::string &uri)
    {
        std::string scheme, path, fragment;
        _ParseURI(uri, scheme, path, fragment);

        std::string filename = _ResolveURI(scheme, path);
        boost::shared_ptr<rapidjson::Document> doc = _OpenDocument(filename);
        if (!doc) {
            throw OPENRAVE_EXCEPTION_FORMAT("failed resolve json document \"%s\"", uri, ORE_InvalidArguments);
        }
        return _ResolveObjectInDocument(doc, fragment);
    }

    void _IndexObjectsInDocument(boost::shared_ptr<rapidjson::Document> doc)
    {
        if (doc->IsObject() && doc->HasMember("objects") && (*doc)["objects"].IsArray()) {
            for (rapidjson::Value::ValueIterator itr = (*doc)["objects"].Begin(); itr != (*doc)["objects"].End(); ++itr) {
                std::string id;
                openravejson::LoadJsonValueByKey(*itr, "id", id);
                if (!id.empty()) {
                    _objects[doc][id] = itr;
                }
            }
        }
    }

    rapidjson::Value::ValueIterator _ResolveObjectInDocument(boost::shared_ptr<rapidjson::Document> doc, const std::string& id)
    {
        if (!!doc) {
            // look for first in doc
            if (id.empty()) {
                if (doc->IsObject() && doc->HasMember("objects") && (*doc)["objects"].IsArray()) {
                    for (rapidjson::Value::ValueIterator itr = (*doc)["objects"].Begin(); itr != (*doc)["objects"].End(); ++itr) {
                        return itr;
                    }
                }
            }

            // use the cache
            if (_objects.find(doc) == _objects.end()) {
                _IndexObjectsInDocument(doc);
            }
            if (_objects[doc].find(id) != _objects[doc].end()) {
                return _objects[doc][id];
            }
        }
        throw OPENRAVE_EXCEPTION_FORMAT("failed resolve object \"%s\" in json document", id, ORE_InvalidArguments);
    }

    /// \brief resolve a uri
    std::string _ResolveURI(const std::string& uri)
    {
        std::string scheme, path, fragment;
        _ParseURI(uri, scheme, path, fragment);

        return _ResolveURI(scheme, path);
    }

    std::string _ResolveURI(const std::string& scheme, const std::string& path)
    {
        if (scheme == "" && path == "")
        {
            return _filename;
        }
        else if (scheme == "file")
        {
            return RaveFindLocalFile(path);
        }
        else if (find(_vOpenRAVESchemeAliases.begin(), _vOpenRAVESchemeAliases.end(), scheme) != _vOpenRAVESchemeAliases.end())
        {
            return RaveFindLocalFile(path);
        }
        
        return "";
    }

    /// \brief open and cache a json document
    boost::shared_ptr<rapidjson::Document> _OpenDocument(const std::string& filename)
    {
        if (_docs.find(filename) != _docs.end()) {
            return _docs[filename];
        }
        std::ifstream ifs(filename.c_str());
        rapidjson::IStreamWrapper isw(ifs);
        boost::shared_ptr<rapidjson::Document> doc;
        doc.reset(new rapidjson::Document);
        rapidjson::ParseResult ok = doc->ParseStream<rapidjson::kParseFullPrecisionFlag>(isw);
        if (!ok) {
            throw OPENRAVE_EXCEPTION_FORMAT("failed parse json document \"%s\"", filename, ORE_InvalidArguments);
        }

        _docs[filename] = doc;
        return doc;
    }

    /// \brief parse and cache a json document
    boost::shared_ptr<rapidjson::Document> _ParseDocument(const std::string& data)
    {
        boost::shared_ptr<rapidjson::Document> doc;
        doc.reset(new rapidjson::Document);
        rapidjson::ParseResult ok = doc->Parse<rapidjson::kParseFullPrecisionFlag>(data.c_str());
        if (!ok) {
            throw OPENRAVE_EXCEPTION_FORMAT0("failed parse json document", ORE_InvalidArguments);
        }

        _docs[""] = doc;
        return doc;
    }

    dReal _fGlobalScale;
    EnvironmentBasePtr _penv;
    std::string _prefix;
    std::string _filename;
    std::string _uri;
    std::vector<std::string> _vOpenRAVESchemeAliases;
    boost::shared_ptr<rapidjson::Document> _doc;
    std::map<std::string, boost::shared_ptr<rapidjson::Document> > _docs; ///< key is filename
    std::map<boost::shared_ptr<rapidjson::Document>, std::map<std::string, rapidjson::Value::ValueIterator> > _objects; ///< key is pointer to doc
};


bool RaveParseJSON(EnvironmentBasePtr penv, const rapidjson::Document& doc, const AttributesList& atts)
{
    JSONReader reader(atts, penv);
    if (!reader.InitFromDocument(doc)) {
        return false;
    }
    return reader.ExtractAll();
}

bool RaveParseJSON(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const rapidjson::Document& doc, const AttributesList& atts)
{
    JSONReader reader(atts, penv);
    if (!reader.InitFromDocument(doc)) {
        return false;
    }
    return reader.ExtractFirst(ppbody);
}

bool RaveParseJSON(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const rapidjson::Document& doc, const AttributesList& atts)
{
    JSONReader reader(atts, penv);
    if (!reader.InitFromDocument(doc)) {
        return false;
    }
    return reader.ExtractFirst(pprobot);
}

bool RaveParseJSONFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts)
{
    JSONReader reader(atts, penv);
    std::string fullFilename = RaveFindLocalFile(filename);
    if (fullFilename.size() == 0 || !reader.InitFromFile(fullFilename)) {
        return false;
    }
    return reader.ExtractAll();
}

bool RaveParseJSONFile(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& filename, const AttributesList& atts)
{
    JSONReader reader(atts, penv);
    std::string fullFilename = RaveFindLocalFile(filename);
    if (fullFilename.size() == 0 || !reader.InitFromFile(fullFilename)) {
        return false;
    }
    return reader.ExtractFirst(ppbody);
}

bool RaveParseJSONFile(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& filename, const AttributesList& atts)
{
    JSONReader reader(atts, penv);
    std::string fullFilename = RaveFindLocalFile(filename);
    if (fullFilename.size() == 0 || !reader.InitFromFile(fullFilename)) {
        return false;
    }
    return reader.ExtractFirst(pprobot);
}

bool RaveParseJSONURI(EnvironmentBasePtr penv, const std::string& uri, const AttributesList& atts)
{
    JSONReader reader(atts, penv);
    if (!reader.InitFromURI(uri)) {
        return false;
    }
    return reader.ExtractAll();
}

bool RaveParseJSONURI(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts)
{
    JSONReader reader(atts, penv);
    if (!reader.InitFromURI(uri)) {
        return false;
    }
    return reader.ExtractOne(ppbody, uri);
}

bool RaveParseJSONURI(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const AttributesList& atts)
{
    JSONReader reader(atts, penv);
    if (!reader.InitFromURI(uri)) {
        return false;
    }
    return reader.ExtractOne(pprobot, uri);
}

bool RaveParseJSONData(EnvironmentBasePtr penv, const std::string& data, const AttributesList& atts)
{
    JSONReader reader(atts, penv);
    if (!reader.InitFromData(data)) {
        return false;
    }
    return reader.ExtractAll();
}

bool RaveParseJSONData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& data, const AttributesList& atts)
{
    JSONReader reader(atts, penv);
    if (!reader.InitFromData(data)) {
        return false;
    }
    return reader.ExtractFirst(ppbody);
}

bool RaveParseJSONData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& data, const AttributesList& atts)
{
    JSONReader reader(atts, penv);
    if (!reader.InitFromData(data)) {
        return false;
    }
    return reader.ExtractFirst(pprobot);
}
}
