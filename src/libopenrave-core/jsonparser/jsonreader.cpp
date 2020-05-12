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
#include <openrave/openrave.h>
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

    /*
    * dofValue format migration
    * dof json value format has changed to a mapping with jointId and value as key, {"jointId": "joint0", "value": 1.234}
    * we need to convert dofvalues from mapping to list for openrave ;
    */
    void _ConvertJointDOFValueFormat(const std::vector<KinBody::JointPtr>& vJoints, std::vector<dReal>& vDOFValues, const rapidjson::Value& rDOFValues) {
        std::map<std::string, dReal> jointDOFValues;
        for(rapidjson::Value::ConstValueIterator itrDOFValue = rDOFValues.Begin(); itrDOFValue != rDOFValues.End(); ++itrDOFValue) {
            std::string jointId;
            dReal dofValue;
            OpenRAVE::JSON::LoadJsonValueByKey(*itrDOFValue, "jointId", jointId);
            OpenRAVE::JSON::LoadJsonValueByKey(*itrDOFValue, "value", dofValue);
            jointDOFValues[jointId] = dofValue;
        }
        vDOFValues.resize(vJoints.size());
        FOREACH(itJoint, vJoints) {
            vDOFValues[(*itJoint)->GetDOFIndex()] = jointDOFValues[(*itJoint)->GetInfo()._id];
        }
    }

    /// \brief Extrat all bodies and add them into environment
    bool ExtractAll() {
        bool allSucceeded = true;
        dReal fUnitScale = _GetUnitScale();

        std::string revision;
        OpenRAVE::JSON::LoadJsonValueByKey(*_doc, "revision", revision);
        if (revision.empty()) {
            revision = "0";
        }
        _penv->SetRevision(revision);
        if (_doc->HasMember("bodies") && (*_doc)["bodies"].IsArray()) {
            std::map<KinBodyPtr, std::vector<KinBody::GrabbedInfoConstPtr>> mapKinBodyGrabbedInfos;
            for (rapidjson::Value::ValueIterator itrBodyValue = (*_doc)["bodies"].Begin(); itrBodyValue != (*_doc)["bodies"].End(); ++itrBodyValue) {
                KinBodyPtr pbody;
                if (_Extract(*itrBodyValue, pbody)) {
                    _penv->Add(pbody, true);
                    // set dof values
                    if (itrBodyValue->HasMember("dofValues") && (*itrBodyValue)["dofValues"].IsObject()) {
                        std::vector<dReal> vDOFValues {};
                        _ConvertJointDOFValueFormat(pbody->GetJoints(), vDOFValues, (*itrBodyValue)["dofValues"]);
                        pbody->SetDOFValues(vDOFValues, KinBody::CLA_Nothing);
                    }

                    // parse grabbed infos
                    if (itrBodyValue->HasMember("grabbed") && (*itrBodyValue)["grabbed"].IsArray()) {
                        for(rapidjson::Value::ValueIterator itGrabInfo = (*itrBodyValue)["grabbed"].Begin(); itGrabInfo != (*itrBodyValue)["grabbed"].End(); ++itGrabInfo){
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
                return _Extract(*itr, pprobot); // TODO: change to iterator
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
                OpenRAVE::JSON::LoadJsonValueByKey(*itr, "uri", bodyUri);
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
                OpenRAVE::JSON::LoadJsonValueByKey(*itr, "uri", bodyUri);
                if (bodyUri.empty()) {
                    std::string id;
                    OpenRAVE::JSON::LoadJsonValueByKey(*itr, "id", id);
                    if (!id.empty()) {
                        bodyUri = "#" + id;
                    }
                }
                if (bodyUri == std::string("#") + fragment) {
                    return _Extract(*itr, pprobot); // TODO: change to iterator
                }
            }
        }
        return false;
    }

protected:
    inline dReal _GetUnitScale()
    {
        std::pair<std::string, dReal> unit = {"meter", 1};
        OpenRAVE::JSON::LoadJsonValueByKey(*_doc, "unit", unit);

        // TODO: for now we just set defautl to ["meter", 1]
        if (unit.first.empty()){
            unit.first = "meter";
            unit.second = 1;
        }

        if (unit.first == "mm") {
            unit.second *= 0.001;
        }
        else if (unit.first == "cm") {
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

    void _ExtractCommon(const rapidjson::Value& bodyValue, KinBody::KinBodyInfo& bodyInfo) {
        OpenRAVE::JSON::LoadJsonValueByKey(bodyValue, "id", bodyInfo._id);
        if (bodyInfo._id.empty()) {
            RAVELOG_WARN("info id is empty");
            bodyInfo._id = "body0";
        }
        int suffix = 1;
        while(_bodyUniqueIds.find(bodyInfo._id) != _bodyUniqueIds.end()) {
            bodyInfo._id = "body" + std::to_string(suffix);
            suffix += 1;
        }
        _bodyUniqueIds.insert(bodyInfo._id);

        // uri
        std::string uri;
        OpenRAVE::JSON::LoadJsonValueByKey(bodyValue, "uri", uri);
        if (uri.empty()) {
            bodyInfo._uri = "#" + bodyInfo._id;
        }
        _CanonicalizeURI(bodyInfo._uri);
    }

    template<typename T>
    void _ExtractEnvInfo(const rapidjson::Value& bodyValue, boost::shared_ptr<T> pbody, dReal fUnitScale) {
        if (!!pbody) {
            _ExtractReadableInterfaces(bodyValue, pbody, fUnitScale);
            std::string bodyName;
            OpenRAVE::JSON::LoadJsonValueByKey(bodyValue, "name", bodyName);
            if (!bodyName.empty()) {
                pbody->SetName(bodyName);
            }

            if (bodyValue.HasMember("transform")) {
                Transform transform;
                OpenRAVE::JSON::LoadJsonValueByKey(bodyValue, "transform", transform);
                transform.trans *= fUnitScale;
                pbody->SetTransform(transform);
            }
        }
    }

    bool _Extract(const rapidjson::Value& bodyValue, KinBodyPtr& pbody)
    {
        // extract for robot
        if (OpenRAVE::JSON::GetJsonValueByKey<bool>(bodyValue, "isRobot")) {
            RobotBasePtr probot;
            if (_Extract(bodyValue, probot)) { // TODO: change robot part to iterator
                pbody = probot;
                return true;
            }
            return false;
        }

        dReal fUnitScale = _GetUnitScale();
        KinBody::KinBodyInfoPtr kinBodyInfo(new KinBody::KinBodyInfo());

        _ExtractCommon(bodyValue, *kinBodyInfo);

        _ExtractLinks(bodyValue, kinBodyInfo->_vLinkInfos, fUnitScale);
        _ExtractJoints(bodyValue, kinBodyInfo->_vJointInfos, fUnitScale);

        KinBodyPtr body = RaveCreateKinBody(_penv, "");
        if (!body->InitFromInfo(kinBodyInfo)) {
            return false;
        }
        _ExtractEnvInfo(bodyValue, body, fUnitScale);
        pbody = body;
        return true;
    }

    bool _Extract(const rapidjson::Value& bodyValue, RobotBasePtr& probot)
    {
        if (!OpenRAVE::JSON::GetJsonValueByKey<bool>(bodyValue, "isRobot")) {
            return false;
        }

        dReal fUnitScale = _GetUnitScale();
        RobotBase::RobotBaseInfoPtr robotBaseInfo(new RobotBase::RobotBaseInfo());
        _ExtractCommon(bodyValue, *robotBaseInfo);

        _ExtractLinks(bodyValue, robotBaseInfo->_vLinkInfos, fUnitScale);
        _ExtractJoints(bodyValue, robotBaseInfo->_vJointInfos, fUnitScale);
        _ExtractManipulators(bodyValue, robotBaseInfo->_vManipInfos, fUnitScale);
        _ExtractAttachedSensors(bodyValue, robotBaseInfo->_vAttachedSensorInfos, fUnitScale);
        _ExtractConnectedBodies(bodyValue, robotBaseInfo->_vConnectedBodyInfos, fUnitScale);
        _ExtractGripperInfos(bodyValue, robotBaseInfo->_vGripperInfos, fUnitScale);

        RobotBasePtr robot = RaveCreateRobot(_penv, "");
        if (!robot->InitFromInfo(robotBaseInfo)) {
            return false;
        }
        _ExtractEnvInfo(bodyValue, robot, fUnitScale);
        probot = robot;
        return true;
    }

    // \brief extract rapidjson value and merge into linkinfos
    void _ExtractLinks(const rapidjson::Value &objectValue, std::vector<KinBody::LinkInfoPtr>& linkinfos, dReal fUnitScale)
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

                rapidjson::Value::ValueIterator connectedBodyObject = _ResolveBodyValue(connectedbodyinfo->_uri);
                _ExtractLinks(*connectedBodyObject, connectedbodyinfo->_vLinkInfos, fUnitScale);
                _ExtractJoints(*connectedBodyObject, connectedbodyinfo->_vJointInfos, fUnitScale);
                _ExtractManipulators(*connectedBodyObject, connectedbodyinfo->_vManipulatorInfos, fUnitScale);
                _ExtractAttachedSensors(*connectedBodyObject, connectedbodyinfo->_vAttachedSensorInfos, fUnitScale);
                connectedbodyinfos.push_back(connectedbodyinfo);
            }
        }
    }

    void _ExtractGripperInfos(const rapidjson::Value& objectValue, std::vector<RobotBase::GripperInfoPtr>& gripperInfos, dReal fUnitScale)
    {
        if (objectValue.HasMember("gripperInfos") && objectValue["gripperInfos"].IsArray()) {
            gripperInfos.reserve(gripperInfos.size() + objectValue["gripperInfos"].Size());
            for (rapidjson::Value::ConstValueIterator itr = objectValue["gripperInfos"].Begin(); itr != objectValue["gripperInfos"].End(); ++itr) {
                RobotBase::GripperInfoPtr gripperinfo(new RobotBase::GripperInfo());
                gripperinfo->DeserializeJSON(*itr, fUnitScale);
                gripperInfos.push_back(gripperinfo);
            }
        }
    }

    void _ExtractReadableInterfaces(const rapidjson::Value &objectValue, InterfaceBasePtr pInterface, dReal fUnitScale)
    {
        if (objectValue.HasMember("readableInterfaces") && objectValue["readableInterfaces"].IsObject()) {
            for (rapidjson::Value::ConstMemberIterator itr = objectValue["readableInterfaces"].MemberBegin(); itr != objectValue["readableInterfaces"].MemberEnd(); itr++) {
                std::string id = itr->name.GetString();
                BaseJSONReaderPtr pReader = RaveCallJSONReader(pInterface->GetInterfaceType(), id, pInterface, AttributesList());
                if(!!pReader) {
                    pReader->DeserializeJSON(itr->value, fUnitScale);
                    JSONReadablePtr pReadable = pReader->GetReadable();
                    if (!!pReadable) {
                        pInterface->SetReadableInterface(id, pReadable);
                    }
                }
                else if(itr->value.IsString()){
                    // TODO: current json data is not able to help distinguish the type. So we try to use string readable if the value is string and no reader is found.
                    StringReadablePtr pReadable(new StringReadable(id, itr->value.GetString()));
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

    rapidjson::Value::ValueIterator _ResolveBodyValue(const std::string &uri)
    {
        std::string scheme, path, fragment;
        _ParseURI(uri, scheme, path, fragment);
        std::string filename = _ResolveURI(scheme, path);
        boost::shared_ptr<rapidjson::Document> doc = _OpenDocument(filename);
        if (!doc) {
            throw OPENRAVE_EXCEPTION_FORMAT("failed resolve json document \"%s\"", uri, ORE_InvalidArguments);
        }
        return _ResolveBodyInDocument(doc, fragment);
    }

    rapidjson::Value::ValueIterator _ResolveBodyInDocument(boost::shared_ptr<rapidjson::Document> doc, const std::string& id)
    {
        if (!!doc) {
            // look for first in doc
            if (id.empty()) {
                if (doc->IsObject() && doc->HasMember("bodies") && (*doc)["bodies"].IsArray()) {
                    for (rapidjson::Value::ValueIterator itr = (*doc)["bodies"].Begin(); itr != (*doc)["bodies"].End(); ++itr) {
                        return itr;
                    }
                }
            }
            // use the cache
            if (_bodies.find(doc) == _bodies.end()) {
                _IndexBodiesInDocument(doc);
            }
            if (_bodies[doc].find(id) != _bodies[doc].end()) {
                return _bodies[doc][id];
            }
        }
        throw OPENRAVE_EXCEPTION_FORMAT("failed resolve object \"%s\" in json document", id, ORE_InvalidArguments);
    }

    void _IndexBodiesInDocument(boost::shared_ptr<rapidjson::Document> doc)
    {
        if (doc->IsObject() && doc->HasMember("bodies") && (*doc)["bodies"].IsArray()) {
            for (rapidjson::Value::ValueIterator itr = (*doc)["bodies"].Begin(); itr != (*doc)["bodies"].End(); ++itr) {
                std::string id;
                OpenRAVE::JSON::LoadJsonValueByKey(*itr, "id", id);
                if (!id.empty()) {
                    _bodies[doc][id] = itr;
                }
            }
        }
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
    std::map<boost::shared_ptr<rapidjson::Document>, std::map<std::string, rapidjson::Value::ValueIterator> > _bodies; ///< key is pointer to doc

    std::set<std::string> _bodyUniqueIds; ///< unique id for bodies in current _doc
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
