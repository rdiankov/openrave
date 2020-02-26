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

        bool ExtractAll() {
            bool allSucceeded = true;
            // extra all bodies and add to env
            std::pair<std::string, dReal> unit;
            LoadJsonValueByKey(*_doc, "unit", unit);
            _penv->SetUnit(unit);

            if (_doc->HasMember("bodies") && (*_doc)["bodies"].IsArray()) {
                for (rapidjson::Value::ValueIterator itr = (*_doc)["bodies"].Begin(); itr != (*_doc)["bodies"].End(); ++itr) {
                    KinBodyPtr pbody;
                    if (_Extract(*itr, pbody)) {
                        _penv->Add(pbody, false);
                    } else {
                        allSucceeded = false;
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
                    LoadJsonValueByKey(*itr, "uri", bodyUri);
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
                    LoadJsonValueByKey(*itr, "uri", bodyUri);
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
            LoadJsonValueByKey(*_doc, "unit", unit);
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

        void _FillBody(rapidjson::Value &body, rapidjson::Document::AllocatorType& allocator)
        {
            std::string uri;
            LoadJsonValueByKey(body, "uri", uri);

            rapidjson::Value::ValueIterator object = _ResolveObject(uri);

            // add the object items into instobjects
            for (rapidjson::Value::MemberIterator it = object->MemberBegin(); it != object->MemberEnd(); ++it) {
                std::string key = it->name.GetString();
                if (key != "" && !body.HasMember(key.c_str())) {
                    rapidjson::Value key(key, allocator);
                    rapidjson::Value value(it->value, allocator);
                    body.AddMember(key, value, allocator);
                }
            }

            // fix the uri
            body.RemoveMember("uri");
            SetJsonValueByKey(body, "uri", _CanonicalizeURI(uri), allocator);
        }

        bool _Extract(const rapidjson::Value &value, KinBodyPtr& pbody)
        {
            if (GetJsonValueByKey<bool>(value, "isRobot")) {
                RobotBasePtr probot;
                if (_Extract(value, probot)) {
                    pbody = probot;
                }
                return false;
            }

            dReal fUnitScale = _GetUnitScale();

            std::string uri;
            LoadJsonValueByKey(value, "uri", uri);

            std::vector<KinBody::LinkInfoConstPtr> linkinfos;
            _ExtractLinks(value, linkinfos, fUnitScale);
            
            std::vector<KinBody::JointInfoConstPtr> jointinfos;
            _ExtractJoints(value, jointinfos, fUnitScale);

            KinBodyPtr body = RaveCreateKinBody(_penv, "");
            if (!body->Init(linkinfos, jointinfos, uri)) {
                return false;
            }

            body->SetName(GetJsonValueByKey<std::string>(value, "name"));
            pbody = body;
            return true;
        }

        bool _Extract(const rapidjson::Value &value, RobotBasePtr& probot)
        {
            if (!GetJsonValueByKey<bool>(value, "isRobot")) {
                return false;
            }

            dReal fUnitScale = _GetUnitScale();

            std::string uri;
            LoadJsonValueByKey(value, "uri", uri);

            std::vector<KinBody::LinkInfoConstPtr> linkinfos;
            _ExtractLinks(value, linkinfos, fUnitScale);
            
            std::vector<KinBody::JointInfoConstPtr> jointinfos;
            _ExtractJoints(value, jointinfos, fUnitScale);

            std::vector<RobotBase::ManipulatorInfoConstPtr> manipinfos;
            _ExtractManipulators(value, manipinfos, fUnitScale);

            std::vector<RobotBase::AttachedSensorInfoConstPtr> attachedsensorinfos;
            _ExtractAttachedSensors(value, attachedsensorinfos, fUnitScale);

            RobotBasePtr robot = RaveCreateRobot(_penv, "");
            if (!robot->Init(linkinfos, jointinfos, manipinfos, attachedsensorinfos, uri)) {
                return false;
            }

            robot->SetName(GetJsonValueByKey<std::string>(value, "name"));
            probot = robot;
            return true;
        }

        bool _ExtractLinks(const rapidjson::Value &value, std::vector<KinBody::LinkInfoConstPtr> &linkinfos, dReal fUnitScale)
        {
            if (value.HasMember("links") && value["links"].IsArray()) {
                linkinfos.reserve(linkinfos.size() + value["links"].MemberCount());
                for (rapidjson::Value::ConstValueIterator itr = value["links"].Begin(); itr != value["links"].End(); ++itr) {
                    KinBody::LinkInfoPtr linkinfo(new KinBody::LinkInfo());
                    linkinfo->DeserializeJSON(*itr, fUnitScale);
                    linkinfos.push_back(linkinfo);
                }
                return true;
            }
            return false;
        }

        bool _ExtractJoints(const rapidjson::Value &value, std::vector<KinBody::JointInfoConstPtr> &jointinfos, dReal fUnitScale)
        {
            if (value.HasMember("joints") && value["joints"].IsArray()) {
                jointinfos.reserve(jointinfos.size() + value["joints"].MemberCount());
                for (rapidjson::Value::ConstValueIterator itr = value["joints"].Begin(); itr != value["joints"].End(); ++itr) {
                    KinBody::JointInfoPtr jointinfo(new KinBody::JointInfo());
                    jointinfo->DeserializeJSON(*itr, fUnitScale);
                    jointinfos.push_back(jointinfo);
                }
                return true;
            }
            return false;
        }

        bool _ExtractManipulators(const rapidjson::Value &value, std::vector<RobotBase::ManipulatorInfoConstPtr> &manipinfos, dReal fUnitScale)
        {
            if (value.HasMember("manipulators") && value["manipulators"].IsArray()) {
                manipinfos.reserve(manipinfos.size() + value["manipulators"].MemberCount());
                for (rapidjson::Value::ConstValueIterator itr = value["manipulators"].Begin(); itr != value["manipulators"].End(); ++itr) {
                    RobotBase::ManipulatorInfoPtr manipinfo(new RobotBase::ManipulatorInfo());
                    manipinfo->DeserializeJSON(*itr, fUnitScale);
                    manipinfos.push_back(manipinfo);
                }
                return true;
            }
            return false;
        }

        bool _ExtractAttachedSensors(const rapidjson::Value &value, std::vector<RobotBase::AttachedSensorInfoConstPtr> &attachedsensorinfos, dReal fUnitScale)
        {
            if (value.HasMember("attachedSensors") && value["attachedSensors"].IsArray()) {
                attachedsensorinfos.reserve(attachedsensorinfos.size() + value["attachedSensors"].MemberCount());
                for (rapidjson::Value::ConstValueIterator itr = value["attachedSensors"].Begin(); itr != value["attachedSensors"].End(); ++itr) {
                    RobotBase::AttachedSensorInfoPtr attachedsensorinfo(new RobotBase::AttachedSensorInfo());
                    attachedsensorinfo->DeserializeJSON(*itr, fUnitScale);
                    attachedsensorinfos.push_back(attachedsensorinfo);
                }
                return true;
            }
            return false;
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
            if (doc->IsObject() && doc->HasMember("objects") && (*doc)["objects"].IsArray())
            {
                for (rapidjson::Value::ValueIterator itr = (*doc)["objects"].Begin(); itr != (*doc)["objects"].End(); ++itr)
                {
                    rapidjson::Value::MemberIterator id = itr->FindMember("id");
                    if (id != itr->MemberEnd() && id->value.IsString()) {
                        std::string idstr = id->value.GetString();
                        if (idstr != "")
                        {
                            _objects[doc][idstr] = itr;
                        }
                    }
                }
            }
        }

        rapidjson::Value::ValueIterator _ResolveObjectInDocument(boost::shared_ptr<rapidjson::Document> doc, const std::string& id)
        {
            if (!!doc)
            {
                if (_objects.find(doc) == _objects.end())
                {
                    _IndexObjectsInDocument(doc);
                }
                if (_objects[doc].find(id) != _objects[doc].end())
                {
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
            else if(find(_vOpenRAVESchemeAliases.begin(), _vOpenRAVESchemeAliases.end(), scheme) != _vOpenRAVESchemeAliases.end())
            {
                return RaveFindLocalFile(path);
                // TODO deal with openrave: or mujin:
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
            rapidjson::ParseResult ok = doc->ParseStream(isw);
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
            rapidjson::ParseResult ok = doc->Parse(data.c_str());
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
