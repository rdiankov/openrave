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
#include <openrave/openravemsgpack.h>
#include <fstream>
#include <rapidjson/writer.h>
#include <rapidjson/ostreamwrapper.h>

namespace OpenRAVE {

class EnvironmentJSONWriter
{
public:
    EnvironmentJSONWriter(const AttributesList& atts, rapidjson::Value& rEnvironment, rapidjson::Document::AllocatorType& allocator) : _rEnvironment(rEnvironment), _allocator(allocator) {
        _serializeOptions = 0;
        FOREACHC(itatt,atts) {
            if( itatt->first == "openravescheme" ) {
                _vForceResolveOpenRAVEScheme = itatt->second;
            }
            else if( itatt->first == "uriHint" ) {
                if( itatt->second == "1" ) {
                    _serializeOptions = ISO_ReferenceUriHint;
                }
            }
        }
    }

    virtual ~EnvironmentJSONWriter() {
    }

    virtual void Write(EnvironmentBasePtr penv) {
        dReal fUnitScale = 1.0;
        EnvironmentBase::EnvironmentBaseInfo info;
        penv->ExtractInfo(info);
        _rEnvironment.SetObject();
        info.SerializeJSON(_rEnvironment, _allocator, fUnitScale, _serializeOptions);
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

    virtual void _Write(const std::list<KinBodyPtr>& listbodies) {
        _rEnvironment.SetObject();
        if (listbodies.size() > 0) {
            EnvironmentBaseConstPtr penv = listbodies.front()->GetEnv();
            OpenRAVE::orjson::SetJsonValueByKey(_rEnvironment, "unit", penv->GetUnit(), _allocator);
            dReal fUnitScale = 1.0;

            FOREACHC(itbody, listbodies) {
                BOOST_ASSERT((*itbody)->GetEnv() == penv);
            }

            rapidjson::Value bodiesValue;
            bodiesValue.SetArray();

            FOREACHC(itBody, listbodies) {
                KinBodyPtr pBody = *itBody;
                rapidjson::Value bodyValue;

                // set dofvalues before serializing body info
                {
                    KinBody::KinBodyStateSaver saver(pBody);
                    vector<dReal> vZeros(pBody->GetDOF(), 0);
                    pBody->SetDOFValues(vZeros, KinBody::CLA_Nothing);
                    pBody->SetTransform(Transform()); // TODO: is this necessary

                    if (!pBody->IsRobot()) {
                        KinBody::KinBodyInfo info;
                        pBody->ExtractInfo(info);
                        info._referenceUri = _CanonicalizeURI(info._referenceUri);
                        info.SerializeJSON(bodyValue, _allocator, fUnitScale);
                    } else {
                        RobotBasePtr pRobot = RaveInterfaceCast<RobotBase>(pBody);
                        RobotBase::RobotBaseInfo info;
                        pRobot->ExtractInfo(info);
                        info._referenceUri = _CanonicalizeURI(info._referenceUri);
                        FOREACH(itConnectedBodyInfo, info._vConnectedBodyInfos) {
                            (*itConnectedBodyInfo)->_uri = _CanonicalizeURI((*itConnectedBodyInfo)->_uri);
                        }
                        info.SerializeJSON(bodyValue, _allocator, fUnitScale, _serializeOptions);
                    }
                }
                // dof value
                std::vector<dReal> vDOFValues;
                pBody->GetDOFValues(vDOFValues);
                if (vDOFValues.size() > 0) {
                    rapidjson::Value dofValues;
                    dofValues.SetArray();
                    dofValues.Reserve(vDOFValues.size(), _allocator);
                    for(size_t iDOF=0; iDOF<vDOFValues.size(); iDOF++) {
                        rapidjson::Value jointDOFValue;
                        KinBody::JointPtr pJoint = pBody->GetJointFromDOFIndex(iDOF);
                        std::string jointName = pJoint->GetName();
                        int jointAxis = iDOF - pJoint->GetDOFIndex();
                        OpenRAVE::orjson::SetJsonValueByKey(jointDOFValue, "jointName", jointName, _allocator);
                        OpenRAVE::orjson::SetJsonValueByKey(jointDOFValue, "jointAxis", jointAxis, _allocator);
                        OpenRAVE::orjson::SetJsonValueByKey(jointDOFValue, "value", vDOFValues[iDOF], _allocator);
                        dofValues.PushBack(jointDOFValue, _allocator);
                    }
                    OpenRAVE::orjson::SetJsonValueByKey(bodyValue, "dofValues", dofValues, _allocator);
                }

                OpenRAVE::orjson::SetJsonValueByKey(bodyValue, "transform", pBody->GetTransform(), _allocator);

                // grabbed info
                std::vector<KinBody::GrabbedInfoPtr> vGrabbedInfo;
                pBody->GetGrabbedInfo(vGrabbedInfo);
                if (vGrabbedInfo.size() > 0) {
                    rapidjson::Value grabbedsValue;
                    grabbedsValue.SetArray();
                    FOREACHC(itgrabbedinfo, vGrabbedInfo) {
                        rapidjson::Value grabbedValue;
                        (*itgrabbedinfo)->SerializeJSON(grabbedValue, _allocator, fUnitScale, _serializeOptions);
                        grabbedsValue.PushBack(grabbedValue, _allocator);
                    }
                    bodyValue.AddMember("grabbed", grabbedsValue, _allocator);
                }

                // finally push to the bodiesValue array if bodyValue is not empty
                if (bodyValue.MemberCount() > 0) {
                    bodiesValue.PushBack(bodyValue, _allocator);
                }
            }

            if (bodiesValue.Size() > 0) {
                _rEnvironment.AddMember("bodies", bodiesValue, _allocator);
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
        if (uri.empty()) {
            return uri;
        }
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

    int _serializeOptions; ///< the serialization options

    rapidjson::Value& _rEnvironment;
    rapidjson::Document::AllocatorType& _allocator;
};

void RaveWriteJSONFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::ofstream ofstream(filename.c_str());
    rapidjson::Document doc(&alloc);

    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(penv);
    OpenRAVE::orjson::DumpJson(doc, ofstream);
}

void RaveWriteJSONFile(KinBodyPtr pbody, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::ofstream ofstream(filename.c_str());
    rapidjson::Document doc(&alloc);

    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(pbody);
    OpenRAVE::orjson::DumpJson(doc, ofstream);
}

void RaveWriteJSONFile(const std::list<KinBodyPtr>& listbodies, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::ofstream ofstream(filename.c_str());
    rapidjson::Document doc(&alloc);

    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(listbodies);
    OpenRAVE::orjson::DumpJson(doc, ofstream);
}

void RaveWriteJSONStream(EnvironmentBasePtr penv, ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(penv);
    OpenRAVE::orjson::DumpJson(doc, os);
}

void RaveWriteJSONStream(KinBodyPtr pbody, ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(pbody);
    OpenRAVE::orjson::DumpJson(doc, os);
}

void RaveWriteJSONStream(const std::list<KinBodyPtr>& listbodies, ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(listbodies);
    OpenRAVE::orjson::DumpJson(doc, os);
}

void RaveWriteJSONMemory(EnvironmentBasePtr penv, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(penv);
    OpenRAVE::orjson::DumpJson(doc, output);
}

void RaveWriteJSONMemory(KinBodyPtr pbody, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(pbody);
    OpenRAVE::orjson::DumpJson(doc, output);
}

void RaveWriteJSONMemory(const std::list<KinBodyPtr>& listbodies, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(listbodies);
    OpenRAVE::orjson::DumpJson(doc, output);
}

void RaveWriteJSON(EnvironmentBasePtr penv, rapidjson::Value& rEnvironment, rapidjson::Document::AllocatorType& allocator, const AttributesList& atts)
{
    EnvironmentJSONWriter jsonwriter(atts, rEnvironment, allocator);
    jsonwriter.Write(penv);
}

void RaveWriteJSON(KinBodyPtr pbody, rapidjson::Value& rEnvironment, rapidjson::Document::AllocatorType& allocator, const AttributesList& atts)
{
    EnvironmentJSONWriter jsonwriter(atts, rEnvironment, allocator);
    jsonwriter.Write(pbody);
}

void RaveWriteJSON(const std::list<KinBodyPtr>& listbodies, rapidjson::Value& rEnvironment, rapidjson::Document::AllocatorType& allocator, const AttributesList& atts)
{
    EnvironmentJSONWriter jsonwriter(atts, rEnvironment, allocator);
    jsonwriter.Write(listbodies);
}

void RaveWriteMsgPackFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::ofstream ofstream(filename.c_str());
    rapidjson::Document doc(&alloc);

    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(penv);
    OpenRAVE::MsgPack::DumpMsgPack(doc, ofstream);
}

void RaveWriteMsgPackFile(KinBodyPtr pbody, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::ofstream ofstream(filename.c_str());
    rapidjson::Document doc(&alloc);

    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(pbody);
    OpenRAVE::MsgPack::DumpMsgPack(doc, ofstream);
}

void RaveWriteMsgPackFile(const std::list<KinBodyPtr>& listbodies, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::ofstream ofstream(filename.c_str());
    rapidjson::Document doc(&alloc);

    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(listbodies);
    OpenRAVE::MsgPack::DumpMsgPack(doc, ofstream);
}

void RaveWriteMsgPackStream(EnvironmentBasePtr penv, ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);

    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(penv);
    OpenRAVE::MsgPack::DumpMsgPack(doc, os);
}

void RaveWriteMsgPackStream(KinBodyPtr pbody, ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);

    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(pbody);
    OpenRAVE::MsgPack::DumpMsgPack(doc, os);
}

void RaveWriteMsgPackStream(const std::list<KinBodyPtr>& listbodies, ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);

    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(listbodies);
    OpenRAVE::MsgPack::DumpMsgPack(doc, os);
}

void RaveWriteMsgPackMemory(EnvironmentBasePtr penv, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(penv);
    OpenRAVE::MsgPack::DumpMsgPack(doc, output);
}

void RaveWriteMsgPackMemory(KinBodyPtr pbody, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);

    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(pbody);
    OpenRAVE::MsgPack::DumpMsgPack(doc, output);
}

void RaveWriteMsgPackMemory(const std::list<KinBodyPtr>& listbodies, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);

    EnvironmentJSONWriter jsonwriter(atts, doc, doc.GetAllocator());
    jsonwriter.Write(listbodies);
    OpenRAVE::MsgPack::DumpMsgPack(doc, output);
}

}

