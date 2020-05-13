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
            else if (itatt->first == "openravescheme") {
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
        _doc = pDoc;
        return true;
    }

    /*
    * dofValue format migration
    * dof json value format has changed to a mapping with jointId and value as key, {"jointId": "joint0", "value": 1.234}
    * we need to convert dofvalues from mapping to list for openrave ;
    */
    void _ConvertJointDOFValueFormat(const std::vector<KinBody::JointPtr>& vJoints, std::vector<dReal>& vDOFValues, const rapidjson::Value& rDOFValues)
    {
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
    bool ExtractAll()
    {
        bool allSucceeded = true;
        dReal fUnitScale = _GetUnitScale();

        uint64_t revision = 0;
        OpenRAVE::JSON::LoadJsonValueByKey(*_doc, "revision", revision);
        _penv->SetRevision(revision);
        if (_doc->HasMember("bodies") && (*_doc)["bodies"].IsArray()) {
            std::map<KinBodyPtr, std::vector<KinBody::GrabbedInfoConstPtr>> mapKinBodyGrabbedInfos;
            for (rapidjson::Value::ValueIterator itrBodyValue = (*_doc)["bodies"].Begin(); itrBodyValue != (*_doc)["bodies"].End(); ++itrBodyValue) {
                KinBodyPtr pbody;
                if (_Extract(*itrBodyValue, pbody)) {
                    _penv->Add(pbody, true);

                    // set dof values
                    if (itrBodyValue->HasMember("dofValues") && (*itrBodyValue)["dofValues"].IsArray()) {
                        std::vector<dReal> vDOFValues {};
                        _ConvertJointDOFValueFormat(pbody->GetJoints(), vDOFValues, (*itrBodyValue)["dofValues"]);
                        pbody->SetDOFValues(vDOFValues, KinBody::CLA_Nothing);
                    }

                    // set transform
                    if (itrBodyValue->HasMember("transform")) {
                        Transform transform;
                        OpenRAVE::JSON::LoadJsonValueByKey(*itrBodyValue, "transform", transform);
                        pbody->SetTransform(transform);
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

    void _EnsureUniqueIdAndUri(KinBody::KinBodyInfo& bodyInfo)
    {
        if (bodyInfo._id.empty()) {
            RAVELOG_WARN("info id is empty");
            bodyInfo._id = "body0";
        }
        int suffix = 1;
        while (_bodyUniqueIds.find(bodyInfo._id) != _bodyUniqueIds.end()) {
            bodyInfo._id = "body" + std::to_string(suffix);
            suffix += 1;
        }
        _bodyUniqueIds.insert(bodyInfo._id);

        bodyInfo._uri = "#" + bodyInfo._id;
        _CanonicalizeURI(bodyInfo._uri);
    }

    template<typename T>
    void _ExtractTransform(const rapidjson::Value& bodyValue, boost::shared_ptr<T> pbody, dReal fUnitScale)
    {
        Transform transform;
        if (bodyValue.HasMember("transform")) {
            OpenRAVE::JSON::LoadJsonValueByKey(bodyValue, "transform", transform);
        }
        transform.trans *= fUnitScale;
        pbody->SetTransform(transform);
    }

    bool _Extract(const rapidjson::Value& bodyValue, KinBodyPtr& pBodyOut)
    {
        // extract for robot
        if (OpenRAVE::JSON::GetJsonValueByKey<bool>(bodyValue, "isRobot")) {
            RobotBasePtr pRobot;
            if (_Extract(bodyValue, pRobot)) { // TODO: change robot part to iterator
                pBodyOut = pRobot;
                return true;
            }
            return false;
        }

        dReal fUnitScale = _GetUnitScale();
        KinBody::KinBodyInfoPtr pKinBodyInfo(new KinBody::KinBodyInfo());
        pKinBodyInfo->DeserializeJSON(bodyValue, fUnitScale);
        _EnsureUniqueIdAndUri(*pKinBodyInfo);

        KinBodyPtr pBody = RaveCreateKinBody(_penv, "");
        if (!pBody->InitFromInfo(pKinBodyInfo)) {
            return false;
        }
        pBody->SetName(pKinBodyInfo->_name);
        _ExtractTransform(bodyValue, pBody, fUnitScale);
        _ExtractReadableInterfaces(bodyValue, pBody, fUnitScale);
        pBodyOut = pBody;
        return true;
    }

    bool _Extract(const rapidjson::Value& bodyValue, RobotBasePtr& pRobotOut)
    {
        if (!OpenRAVE::JSON::GetJsonValueByKey<bool>(bodyValue, "isRobot")) {
            return false;
        }

        dReal fUnitScale = _GetUnitScale();
        RobotBase::RobotBaseInfoPtr pRobotBaseInfo(new RobotBase::RobotBaseInfo());
        pRobotBaseInfo->DeserializeJSON(bodyValue, fUnitScale);
        _EnsureUniqueIdAndUri(*pRobotBaseInfo);

        RobotBasePtr pRobot = RaveCreateRobot(_penv, "");
        if (!pRobot->InitFromInfo(pRobotBaseInfo)) {
            return false;
        }
        pRobot->SetName(pRobotBaseInfo->_name);
        _ExtractTransform(bodyValue, pRobot, fUnitScale);
        _ExtractReadableInterfaces(bodyValue, pRobot, fUnitScale);
        pRobotOut = pRobot;
        return true;
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
        std::ifstream ifs(filename.c_str());
        rapidjson::IStreamWrapper isw(ifs);
        boost::shared_ptr<rapidjson::Document> doc;
        doc.reset(new rapidjson::Document);
        rapidjson::ParseResult ok = doc->ParseStream<rapidjson::kParseFullPrecisionFlag>(isw);
        if (!ok) {
            throw OPENRAVE_EXCEPTION_FORMAT("failed parse json document \"%s\"", filename, ORE_InvalidArguments);
        }
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
        return doc;
    }

    dReal _fGlobalScale;
    EnvironmentBasePtr _penv;
    std::string _prefix;
    std::string _filename;
    std::string _uri;
    std::vector<std::string> _vOpenRAVESchemeAliases;
    boost::shared_ptr<rapidjson::Document> _doc;

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
