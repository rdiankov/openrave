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
    void _SerializeInfoJSON(const T& currentInfo, const std::vector<boost::shared_ptr<T>>& referenceInfos, rapidjson::Value& value, dReal fUnitScale=1.0, int options=0) {
        typename std::vector<boost::shared_ptr<T>>::const_iterator itRef = referenceInfos.begin();
        for (itRef = referenceInfos.begin(); itRef != referenceInfos.end(); itRef++) {
            if ((*itRef)->_id == currentInfo._id) {
                break;
            }
        }
        if (referenceInfos.size() > 0 && itRef != referenceInfos.end()) {
            // SerializeDiffJSON(currentInfo, **itRef, value, _allocator);
            currentInfo.SerializeDiffJSON(value, **itRef, _allocator, fUnitScale, options);
        }
        else {
            currentInfo.SerializeJSON(value, _allocator, fUnitScale, options);
        }
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
                    rapidjson::Value dofValues;
                    dofValues.SetArray();
                    dofValues.Reserve(vDOFValues.size(), _allocator);
                    for(size_t iDOF=0; iDOF<vDOFValues.size(); iDOF++) {
                        rapidjson::Value jointDOFValue;
                        KinBody::JointPtr pJoint = pbody->GetJointFromDOFIndex(iDOF);
                        std::string jointId = pJoint->GetInfo()._id;
                        if (jointId.empty()) {
                            jointId = pJoint->GetInfo()._name;
                        }
                        OpenRAVE::JSON::SetJsonValueByKey(jointDOFValue, "jointId", jointId, _allocator);
                        OpenRAVE::JSON::SetJsonValueByKey(jointDOFValue, "value", vDOFValues[iDOF], _allocator);
                        dofValues.PushBack(jointDOFValue, _allocator);
                    }
                    OpenRAVE::JSON::SetJsonValueByKey(bodyValue, "dofValues", dofValues, _allocator);
                }

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

                std::string id = (*itbody)->GetInfo()._id;
                if (id.empty()) {
                    id = str(boost::format("body%d_motion")%_mapBodyIds[pbody->GetEnvironmentId()]);
                }
                OpenRAVE::JSON::SetJsonValueByKey(bodyValue, "id", id, _allocator);
                OpenRAVE::JSON::SetJsonValueByKey(bodyValue, "name", pbody->GetName(), _allocator);

                // referenceUri
                if (!!pbody && !!pbody->GetInfo()._referenceInfo) {
                    OpenRAVE::JSON::SetJsonValueByKey(bodyValue, "referenceUri", (*itbody)->GetInfo()._referenceUri, _allocator);
                }

                const KinBody::KinBodyInfo& bodyInfo = pbody->GetInfo();
                // links
                {
                    rapidjson::Value linksValue;
                    linksValue.SetArray();
                    // get current linkinfoptr vector
                    FOREACHC(itlink, pbody->GetLinks()) {
                        KinBody::LinkInfoPtr pLinkInfo = boost::make_shared<KinBody::LinkInfo>((*itlink)->UpdateAndGetInfo());
                        rapidjson::Value value;
                        if (!!bodyInfo._referenceInfo) {
                            _SerializeInfoJSON(*pLinkInfo, bodyInfo._referenceInfo->_vLinkInfos, value);
                        }
                        else {
                            pLinkInfo->SerializeJSON(value, _allocator);
                        }
                        if (value.MemberCount() > 0){
                            linksValue.PushBack(value, _allocator);
                        }
                    }

                    if (linksValue.Size() > 0) {
                        bodyValue.AddMember("links", linksValue, _allocator);
                    }
                }

                // joints
                if (pbody->GetJoints().size() + pbody->GetPassiveJoints().size() > 0)
                {
                    rapidjson::Value jointsValue;
                    jointsValue.SetArray();

                    const std::vector<KinBody::JointPtr>& vJoints = pbody->GetJoints();
                    const std::vector<KinBody::JointPtr>& vPassiveJoints = pbody->GetPassiveJoints();

                    typename std::vector<KinBody::JointPtr>::const_iterator itJoint = vJoints.begin();
                    for(;itJoint!=vJoints.end(); itJoint++){
                        KinBody::JointInfoPtr pJointInfo = boost::make_shared<KinBody::JointInfo>((*itJoint)->GetInfo());
                        rapidjson::Value value;
                        if (!!bodyInfo._referenceInfo) {
                            _SerializeInfoJSON(*pJointInfo, bodyInfo._referenceInfo->_vJointInfos, value);
                        }
                        else {
                            pJointInfo->SerializeJSON(value, _allocator);
                        }
                        if (value.MemberCount() > 0) {
                            jointsValue.PushBack(value, _allocator);
                        }
                    }

                    typename std::vector<KinBody::JointPtr>::const_iterator itPassiveJoint = vPassiveJoints.begin();
                    for(;itPassiveJoint!=vPassiveJoints.end(); itPassiveJoint++){
                        KinBody::JointInfoPtr pJointInfo = boost::make_shared<KinBody::JointInfo>((*itPassiveJoint)->GetInfo());
                        rapidjson::Value value;
                        if (!!bodyInfo._referenceInfo) {
                            _SerializeInfoJSON(*pJointInfo, bodyInfo._referenceInfo->_vJointInfos, value);
                        }
                        else {
                            pJointInfo->SerializeJSON(value, _allocator);
                        }
                        if (value.MemberCount() > 0) {
                            jointsValue.PushBack(value, _allocator);
                        }
                    }

                    if (jointsValue.Size() > 0) {
                        bodyValue.AddMember("joints", jointsValue, _allocator);
                    }
                }

                // robot
                if (pbody->IsRobot()) {
                    RobotBasePtr probot = RaveInterfaceCast<RobotBase>(pbody);
                    const RobotBase::RobotBaseInfo& robotInfo = probot->GetInfo();
                    // manipulators
                    if (probot->GetManipulators().size() > 0) {
                        rapidjson::Value manipulatorsValue;
                        manipulatorsValue.SetArray();

                        FOREACHC(itmanip, probot->GetManipulators()) {
                            RobotBase::ManipulatorInfoPtr pManipInfo = boost::make_shared<RobotBase::ManipulatorInfo>((*itmanip)->GetInfo());
                            rapidjson::Value value;
                            if (!!robotInfo._referenceInfo) {
                                _SerializeInfoJSON(*pManipInfo, robotInfo._referenceInfo->_vManipInfos, value);
                            }
                            else {
                                pManipInfo->SerializeJSON(value, _allocator);
                            }
                            manipulatorsValue.PushBack(value, _allocator);
                        }
                        if (manipulatorsValue.Size() > 0) {
                            bodyValue.AddMember("manipulators", manipulatorsValue, _allocator);
                        }
                    }
                    // attachedsensors
                    if (probot->GetAttachedSensors().size() > 0) {
                        rapidjson::Value attachedSensorsValue;
                        attachedSensorsValue.SetArray();
                        FOREACHC(itattachedsensor, probot->GetAttachedSensors()) {
                            RobotBase::AttachedSensorInfoPtr pAttachedSensorInfo = boost::make_shared<RobotBase::AttachedSensorInfo>((*itattachedsensor)->GetInfo());
                            rapidjson::Value value;
                            if (!!robotInfo._referenceInfo) {
                                _SerializeInfoJSON(*pAttachedSensorInfo, robotInfo._referenceInfo->_vAttachedSensorInfos, value);
                            }
                            else {
                                pAttachedSensorInfo->SerializeJSON(value, _allocator);
                            }
                            attachedSensorsValue.PushBack(value, _allocator);
                        }

                        if (attachedSensorsValue.Size() > 0) {
                            bodyValue.AddMember("attachedSensors", attachedSensorsValue, _allocator);
                        }
                    }
                    // connectedbodies
                    if (probot->GetConnectedBodies().size() > 0) {
                        rapidjson::Value connectedBodiesValue;
                        connectedBodiesValue.SetArray();

                        FOREACHC(itconnectedbody, probot->GetConnectedBodies()) {
                            RobotBase::ConnectedBodyInfoPtr pConnectedBodyInfo = boost::make_shared<RobotBase::ConnectedBodyInfo>((*itconnectedbody)->GetInfo());
                            rapidjson::Value value;
                            if (!!robotInfo._referenceInfo) {
                                _SerializeInfoJSON(*pConnectedBodyInfo, robotInfo._referenceInfo->_vConnectedBodyInfos, value);
                            }
                            else {
                                pConnectedBodyInfo->SerializeJSON(value, _allocator);
                            }
                            connectedBodiesValue.PushBack(value, _allocator);
                        }
                        if (connectedBodiesValue.Size() > 0) {
                            bodyValue.AddMember("connectedBodies", connectedBodiesValue, _allocator);
                        }
                    }

                    // gripperinfo
                    if (probot->GetGripperInfos().size() > 0) {
                        rapidjson::Value gripperInfosValue;
                        gripperInfosValue.SetArray();
                        FOREACHC(itgripperinfo, probot->GetGripperInfos()) {
                            rapidjson::Value value;
                            if (!!robotInfo._referenceInfo) {
                                _SerializeInfoJSON(**itgripperinfo, robotInfo._referenceInfo->_vGripperInfos, value);
                            }
                            else {
                                (*itgripperinfo)->SerializeJSON(value, _allocator);
                            }
                            gripperInfosValue.PushBack(value, _allocator);
                        }

                        if (gripperInfosValue.Size() > 0) {
                            bodyValue.AddMember("gripperInfos", gripperInfosValue, _allocator);
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
