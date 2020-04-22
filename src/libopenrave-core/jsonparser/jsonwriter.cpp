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
#include <fstream>
#include <rapidjson/writer.h>
#include <rapidjson/ostreamwrapper.h>

namespace OpenRAVE {

class EnvironmentJSONWriter
{
public:
    EnvironmentJSONWriter(const AttributesList& atts, rapidjson::Value& rScene, rapidjson::Document::AllocatorType& allocator) : _bExternalRefAllBodies(false), _rScene(rScene), _allocator(allocator) {
        FOREACHC(itatt,atts) {
            if( itatt->first == "externalref" ) {
                if( itatt->second == "*" ) {
                    _bExternalRefAllBodies = true;
                }
                else {
                    stringstream ss(itatt->second);
                    std::list<string> newelts((istream_iterator<string>(ss)), istream_iterator<string>());
                    _listExternalRefExports.splice(_listExternalRefExports.end(),newelts);
                }
            }
            else if( itatt->first == "openravescheme" ) {
                _vForceResolveOpenRAVEScheme = itatt->second;
            }
        }
    }

    virtual ~EnvironmentJSONWriter() {
    }

    virtual void Write(EnvironmentBasePtr penv) {
        std::vector<KinBodyPtr> vbodies;
        penv->GetBodies(vbodies);
        std::list<KinBodyPtr> listbodies(vbodies.begin(), vbodies.end());
        _Write(listbodies);
    }

    virtual void Write(KinBodyPtr pbody) {
        std::list<KinBodyPtr> listbodies;
        listbodies.push_back(pbody);
        _Write(listbodies);
    }

    virtual void Write(const std::list<KinBodyPtr>& listbodies) {
        _Write(listbodies);
    }

protected:

    // \brief find out missing part in current info based on referenced info and set them as deleted in valueArray
    template<typename T>
    void _SerializeDeletedInfoJSON(const std::vector<T>& current, const std::vector<T>& referenced, rapidjson::Value& valueArray) {
        FOREACH(rItr, referenced) {
            bool found = false;
            FOREACH(itr, current) {
                if ((*rItr)->_id == (*itr)->_id) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                rapidjson::Value value;
                OpenRAVE::JSON::SetJsonValueByKey(value, "id", (*rItr)->_id, _allocator);
                OpenRAVE::JSON::SetJsonValueByKey(value, "__deleted__", true, _allocator);
                valueArray.PushBack(value, _allocator);
            }
        }
    }

    template<typename T>
    void _SerializeInfoJSON(const std::vector<T>& currentInfos, const std::vector<T>& referenceInfos, rapidjson::Value& valueArray) {
        FOREACHC(it, currentInfos) {
            rapidjson::Value value;
            auto itRef = std::find_if(referenceInfos.begin(), referenceInfos.end(), [&](T info){return info->_id == (*it)->_id;});
            if ( itRef != referenceInfos.end()) {
                SerializeDiffJSON(**it, **itRef, value, _allocator);
            }
            else {
                (*it)->SerializeJSON(value, _allocator);
            }
            valueArray.PushBack(value, _allocator);
        }

        _SerializeDeletedInfoJSON(currentInfos, referenceInfos, valueArray);
    }

    virtual void _Write(const std::list<KinBodyPtr>& listbodies) {
        _rScene.SetObject();
        _mapBodyIds.clear();
        if (listbodies.size() > 0) {
            EnvironmentBaseConstPtr penv = listbodies.front()->GetEnv();
            OpenRAVE::JSON::SetJsonValueByKey(_rScene, "unit", penv->GetUnit(), _allocator);

            int globalId = 0;
            FOREACHC(itbody, listbodies) {
                BOOST_ASSERT((*itbody)->GetEnv() == penv);
                BOOST_ASSERT(_mapBodyIds.find((*itbody)->GetEnvironmentId()) == _mapBodyIds.end());
                _mapBodyIds[(*itbody)->GetEnvironmentId()] = globalId++;
            }

            rapidjson::Value bodiesValue;
            bodiesValue.SetArray();

            FOREACHC(itbody,listbodies) {
                KinBodyPtr pbody = *itbody;

                rapidjson::Value bodyValue;
                bodyValue.SetObject();

                // dof value
                std::vector<dReal> vDOFValues;
                pbody->GetDOFValues(vDOFValues);
                if (vDOFValues.size() > 0) {
                    OpenRAVE::JSON::SetJsonValueByKey(bodyValue, "dofValues", vDOFValues, _allocator);
                }

                // state saver

                KinBody::KinBodyStateSaver saver(pbody);
                vector<dReal> vZeros(pbody->GetDOF());
                pbody->SetDOFValues(vZeros, KinBody::CLA_Nothing);
                pbody->SetTransform(Transform());

                // grabbed info
                std::vector<KinBody::GrabbedInfoPtr> vGrabbedInfo;
                pbody->GetGrabbedInfo(vGrabbedInfo);
                if (vGrabbedInfo.size() > 0) {
                    rapidjson::Value grabbedsValue;
                    grabbedsValue.SetArray();
                    FOREACHC(itgrabbedinfo, vGrabbedInfo) {
                        rapidjson::Value grabbedValue;
                        (*itgrabbedinfo)->SerializeJSON(grabbedValue, _allocator);
                        grabbedsValue.PushBack(grabbedValue, _allocator);
                    }
                    bodyValue.AddMember("grabbed", grabbedsValue, _allocator);
                }

                // id
                std::string id = str(boost::format("body%d_motion")%_mapBodyIds[pbody->GetEnvironmentId()]);
                OpenRAVE::JSON::SetJsonValueByKey(bodyValue, "_id", id, _allocator);

                // links
                {
                    rapidjson::Value linksValue;
                    linksValue.SetArray();

                    // get current linkinfoptr vector
                    std::vector<KinBody::LinkInfoPtr> currentLinkInfos;
                    currentLinkInfos.reserve(pbody->GetLinks().size());
                    FOREACHC(itlink, pbody->GetLinks()) {
                        currentLinkInfos.push_back(boost::make_shared<KinBody::LinkInfo>((*itlink)->GetInfo()));
                    }
                    _SerializeInfoJSON(currentLinkInfos, pbody->GetInfo()._vLinkInfos, linksValue);

                    if (linksValue.Size() > 0) {
                        bodyValue.AddMember("links", linksValue, _allocator);
                    }
                }

                // joints
                if (pbody->GetJoints().size() + pbody->GetPassiveJoints().size() > 0)
                {
                    rapidjson::Value jointsValue;
                    jointsValue.SetArray();

                    std::vector<KinBody::JointInfoPtr> currentJointInfos;
                    FOREACHC(itjoint, pbody->GetJoints()) {
                        currentJointInfos.push_back(boost::make_shared<KinBody::JointInfo>((*itjoint)->GetInfo()));
                    }

                    FOREACHC(itjoint, pbody->GetPassiveJoints()) {
                        currentJointInfos.push_back(boost::make_shared<KinBody::JointInfo>((*itjoint)->GetInfo()));
                    }

                    _SerializeInfoJSON(currentJointInfos, pbody->GetInfo()._vJointInfos, jointsValue);

                    if (jointsValue.Size() > 0) {
                        bodyValue.AddMember("joints", jointsValue, _allocator);
                    }
                }

                // robot realted structues
                if (pbody->IsRobot()) {
                    RobotBasePtr probot = RaveInterfaceCast<RobotBase>(pbody);
                    if (probot->GetManipulators().size() > 0) {
                        rapidjson::Value manipulatorsValue;
                        manipulatorsValue.SetArray();

                        std::vector<RobotBase::ManipulatorInfoPtr> currentManipulatorInfos;
                        FOREACHC(itmanip, probot->GetManipulators()) {
                            currentManipulatorInfos.push_back(boost::make_shared<RobotBase::ManipulatorInfo>((*itmanip)->GetInfo()));
                        }

                        _SerializeInfoJSON(currentManipulatorInfos, probot->GetInfo()._vManipInfos, manipulatorsValue);

                        if (manipulatorsValue.Size() > 0) {
                            bodyValue.AddMember("manipulators", manipulatorsValue, _allocator);
                        }
                    }

                    if (probot->GetAttachedSensors().size() > 0) {
                        rapidjson::Value attachedSensorsValue;
                        attachedSensorsValue.SetArray();

                        std::vector<RobotBase::AttachedSensorInfoPtr> currentAttachedSensorInfos;
                        FOREACHC(itattachedsensor, probot->GetAttachedSensors()) {
                            currentAttachedSensorInfos.push_back(boost::make_shared<RobotBase::AttachedSensorInfo>((*itattachedsensor)->GetInfo()));
                        }
                        _SerializeInfoJSON(currentAttachedSensorInfos, probot->GetInfo()._vAttachedSensorInfos, attachedSensorsValue);

                        if (attachedSensorsValue.Size() > 0) {
                            bodyValue.AddMember("attachedSensors", attachedSensorsValue, _allocator);
                        }
                    }

                    if (probot->GetConnectedBodies().size() > 0) {
                        rapidjson::Value connectedBodiesValue;
                        connectedBodiesValue.SetArray();

                        std::vector<RobotBase::ConnectedBodyInfoPtr> currentConnectedBodyInfos;
                        FOREACHC(itconnectedbody, probot->GetConnectedBodies()) {
                            currentConnectedBodyInfos.push_back(boost::make_shared<RobotBase::ConnectedBodyInfo>((*itconnectedbody)->GetInfo()));
                        }
                        _SerializeInfoJSON(currentConnectedBodyInfos, probot->GetInfo()._vConnectedBodyInfos, connectedBodiesValue);
                        if (connectedBodiesValue.Size() > 0) {
                            bodyValue.AddMember("connectedBodies", connectedBodiesValue, _allocator);
                        }

                    }
                    OpenRAVE::JSON::SetJsonValueByKey(bodyValue, "isRobot", true, _allocator);
                }

                // readable interface
                if (pbody->GetReadableInterfaces().size() > 0) {
                    rapidjson::Value readableInterfacesValue;
                    readableInterfacesValue.SetObject();
                    FOREACHC(it, pbody->GetReadableInterfaces()) {
                        JSONReadablePtr pReadable = OPENRAVE_DYNAMIC_POINTER_CAST<JSONReadable>(it->second);
                        if (!!pReadable) {
                            rapidjson::Value readableValue;
                            pReadable->SerializeJSON(readableValue, _allocator);
                            readableInterfacesValue.AddMember(rapidjson::Value(it->first.c_str(), _allocator).Move(), readableValue, _allocator);
                        }
                    }
                    if (readableInterfacesValue.MemberCount() > 0) {
                        bodyValue.AddMember("readableInterfaces", readableInterfacesValue, _allocator);
                    }
                }
                // finally push to the bodiesValue array if bodyValue is not empty

                if (bodyValue.MemberCount() > 0) {
                    // name and transform
                    OpenRAVE::JSON::SetJsonValueByKey(bodyValue, "name", pbody->GetName(), _allocator);
                    OpenRAVE::JSON::SetJsonValueByKey(bodyValue, "transform", pbody->GetTransform(), _allocator);
                    bodiesValue.PushBack(bodyValue, _allocator);
                }
            }

            if (bodiesValue.Size() > 0) {
                _rScene.AddMember("bodies", bodiesValue, _allocator);
            }
        }
    }

    /// \brief checks if a body can be written externally
    virtual bool _CheckForExternalWrite(KinBodyPtr pbody)
    {
        if( !_bExternalRefAllBodies && find(_listExternalRefExports.begin(),_listExternalRefExports.end(),pbody->GetName()) == _listExternalRefExports.end() ) {
            // user doesn't want to use external refs
            return false;
        }

        // TODO: need to check if the external reference is refering to a json file
        return pbody->GetURI().size() > 0;
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
            scheme = path.substr(0, colonindex);
            path = path.substr(colonindex + 1);
        }
    }

    std::string _CanonicalizeURI(const std::string& uri)
    {
        std::string scheme, path, fragment;
        _ParseURI(uri, scheme, path, fragment);

        if (_vForceResolveOpenRAVEScheme.size() > 0 && scheme == "file") {
            // check if inside an openrave path, and if so, return the openrave relative directory instead using "openrave:"
            std::string filename;
            if (RaveInvertFileLookup(filename, path)) {
                path = "/" + filename;
                scheme = _vForceResolveOpenRAVEScheme;
            }
        }

        // TODO: fix other scheme.

        // fix extension, replace dae with json
        // this is done for ease of migration
        size_t len = path.size();
        if (len >= sizeof(".dae") - 1) {
            if (path[len - 4] == '.' && ::tolower(path[len - 3]) == 'd' && ::tolower(path[len - 2]) == 'a' && ::tolower(path[len - 1]) == 'e') {
                path = path.substr(0, path.size() - (sizeof(".dae") - 1)) + ".json";
            }
        }

        std::string newuri = scheme + ":" + path;
        if (fragment.size() > 0) {
            newuri += "#" + fragment;
        }
        return newuri;
    }

    std::string _vForceResolveOpenRAVEScheme; ///< if specified, writer will attempt to convert a local system URI (**file:/**) to a a relative path with respect to $OPENRAVE_DATA paths and use **customscheme** as the scheme
    std::list<std::string> _listExternalRefExports; ///< body names to try to export externally
    bool _bExternalRefAllBodies; ///< if true, attempts to externally write all bodies

    std::map<int, int> _mapBodyIds; ///< map from body environment id to unique json ids

    rapidjson::Value& _rScene;
    rapidjson::Document::AllocatorType& _allocator;
};

void RaveWriteJSONFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts)
{
    std::ofstream ofstream(filename.c_str());
    rapidjson::Document doc;

    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(penv);
    OpenRAVE::JSON::DumpJson(doc, ofstream);
}

void RaveWriteJSONFile(KinBodyPtr pbody, const std::string& filename, const AttributesList& atts)
{
    std::ofstream ofstream(filename.c_str());
    rapidjson::Document doc;

    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(pbody);
    OpenRAVE::JSON::DumpJson(doc, ofstream);
}

void RaveWriteJSONFile(const std::list<KinBodyPtr>& listbodies, const std::string& filename, const AttributesList& atts)
{
    std::ofstream ofstream(filename.c_str());
    rapidjson::Document doc;

    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(listbodies);
    OpenRAVE::JSON::DumpJson(doc, ofstream);
}
void RaveWriteJSONStream(EnvironmentBasePtr penv, ostream& os, const AttributesList& atts)
{
    rapidjson::Document doc;

    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(penv);
    OpenRAVE::JSON::DumpJson(doc, os);
}
void RaveWriteJSONStream(KinBodyPtr pbody, ostream& os, const AttributesList& atts)
{
    rapidjson::Document doc;

    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(pbody);
    OpenRAVE::JSON::DumpJson(doc, os);
}
void RaveWriteJSONStream(const std::list<KinBodyPtr>& listbodies, ostream& os, const AttributesList& atts)
{
    rapidjson::Document doc;

    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(listbodies);
    OpenRAVE::JSON::DumpJson(doc, os);
}

void RaveWriteJSONMemory(EnvironmentBasePtr penv, std::vector<char>& output, const AttributesList& atts)
{
    rapidjson::Document doc;

    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(penv);
    OpenRAVE::JSON::DumpJson(doc, output);
}

void RaveWriteJSONMemory(KinBodyPtr pbody, std::vector<char>& output, const AttributesList& atts)
{
    rapidjson::Document doc;

    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(pbody);
    OpenRAVE::JSON::DumpJson(doc, output);
}

void RaveWriteJSONMemory(const std::list<KinBodyPtr>& listbodies, std::vector<char>& output, const AttributesList& atts)
{
    rapidjson::Document doc;

    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(listbodies);
    OpenRAVE::JSON::DumpJson(doc, output);
}

void RaveWriteJSON(EnvironmentBasePtr penv, rapidjson::Value& rScene, rapidjson::Document::AllocatorType& allocator, const AttributesList& atts)
{
    EnvironmentJSONWriter jsonwriter(atts, rScene, allocator);
    jsonwriter.Write(penv);
}

void RaveWriteJSON(KinBodyPtr pbody, rapidjson::Value& rScene, rapidjson::Document::AllocatorType& allocator, const AttributesList& atts)
{
    EnvironmentJSONWriter jsonwriter(atts, rScene, allocator);
    jsonwriter.Write(pbody);
}

void RaveWriteJSON(const std::list<KinBodyPtr>& listbodies, rapidjson::Value& rScene, rapidjson::Document::AllocatorType& allocator, const AttributesList& atts)
{
    EnvironmentJSONWriter jsonwriter(atts, rScene, allocator);
    jsonwriter.Write(listbodies);
}

}
