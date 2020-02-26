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

class JSONWriter {
public:

    JSONWriter(const AttributesList& atts, rapidjson::Document &doc) : _bExternalRefAllBodies(false), _doc(doc) {
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

    virtual ~JSONWriter() {
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

    virtual void _Write(const std::list<KinBodyPtr>& listbodies) {
        _doc.SetObject();
        _mapBodyIds.clear();
        if (listbodies.size() > 0) {
            EnvironmentBaseConstPtr penv = listbodies.front()->GetEnv();
            SetJsonValueByKey(_doc, "unit", penv->GetUnit(), _doc.GetAllocator());

            int globalId = 0;
            FOREACHC(itbody, listbodies) {
                BOOST_ASSERT((*itbody)->GetEnv() == penv);
                BOOST_ASSERT(_mapBodyIds.find((*itbody)->GetEnvironmentId()) == _mapBodyIds.end());
                _mapBodyIds[(*itbody)->GetEnvironmentId()] = globalId++;
            }

            rapidjson::Value bodiesValue;
            bodiesValue.SetArray();

            rapidjson::Value objectsValue;
            objectsValue.SetArray();

            FOREACHC(itbody,listbodies) {
                KinBodyPtr pbody = *itbody;

                rapidjson::Value bodyValue;
                bodyValue.SetObject();
                SetJsonValueByKey(bodyValue, "name", pbody->GetName(), _doc.GetAllocator());
                SetJsonValueByKey(bodyValue, "transform", pbody->GetTransform(), _doc.GetAllocator());

                std::vector<dReal> vDOFValues;
                pbody->GetDOFValues(vDOFValues);
                if (vDOFValues.size() > 0) {
                    SetJsonValueByKey(bodyValue, "dofValues", vDOFValues, _doc.GetAllocator());
                }

                std::vector<KinBody::GrabbedInfoPtr> vGrabbedInfo;
                pbody->GetGrabbedInfo(vGrabbedInfo);
                if (vGrabbedInfo.size() > 0) {
                    rapidjson::Value grabbedsValue;
                    grabbedsValue.SetArray();
                    FOREACHC(itgrabbedinfo, vGrabbedInfo) {
                        rapidjson::Value grabbedValue;
                        (*itgrabbedinfo)->SerializeJSON(grabbedValue, _doc.GetAllocator());
                        grabbedsValue.PushBack(grabbedValue, _doc.GetAllocator());
                    }
                    bodyValue.AddMember("grabbed", grabbedsValue, _doc.GetAllocator());
                }

                if (_CheckForExternalWrite(pbody)) {
                    // for external references, need to canonicalize the uri
                    std::string uri = _CanonicalizeURI(pbody->GetURI());
                    SetJsonValueByKey(bodyValue, "uri", uri, _doc.GetAllocator());
                    bodiesValue.PushBack(bodyValue, _doc.GetAllocator());
                    continue;
                }

                // non-external reference
                std::string id = str(boost::format("body%d_motion")%_mapBodyIds[pbody->GetEnvironmentId()]);
                SetJsonValueByKey(bodyValue, "uri", std::string("#") + id, _doc.GetAllocator());
                bodiesValue.PushBack(bodyValue, _doc.GetAllocator());

                rapidjson::Value objectValue;
                objectValue.SetObject();
                SetJsonValueByKey(objectValue, "id", id, _doc.GetAllocator());
                SetJsonValueByKey(objectValue, "name", pbody->GetName(), _doc.GetAllocator());

                {
                    rapidjson::Value linksValue;
                    linksValue.SetArray();
                    FOREACHC(itlink, pbody->GetLinks()) {
                        rapidjson::Value linkValue;
                        (*itlink)->GetInfo().SerializeJSON(linkValue, _doc.GetAllocator());
                        linksValue.PushBack(linkValue, _doc.GetAllocator());
                    }
                    objectValue.AddMember("links", linksValue, _doc.GetAllocator());
                }

                if (pbody->GetJoints().size() + pbody->GetPassiveJoints().size() > 0)
                {
                    rapidjson::Value jointsValue;
                    jointsValue.SetArray();
                    FOREACHC(itjoint, pbody->GetJoints()) {
                        rapidjson::Value jointValue;
                        (*itjoint)->GetInfo().SerializeJSON(jointValue, _doc.GetAllocator());
                        jointsValue.PushBack(jointValue, _doc.GetAllocator());
                    }
                    FOREACHC(itjoint, pbody->GetPassiveJoints()) {
                        rapidjson::Value jointValue;
                        (*itjoint)->GetInfo().SerializeJSON(jointValue, _doc.GetAllocator());
                        jointsValue.PushBack(jointValue, _doc.GetAllocator());
                    }
                    objectValue.AddMember("joints", jointsValue, _doc.GetAllocator());
                }

                if (pbody->IsRobot()) {
                    RobotBasePtr probot = RaveInterfaceCast<RobotBase>(pbody);
                    if (probot->GetManipulators().size() > 0) {
                        rapidjson::Value manipulatorsValue;
                        manipulatorsValue.SetArray();
                        FOREACHC(itmanip, probot->GetManipulators()) {
                            rapidjson::Value manipulatorValue;
                            (*itmanip)->GetInfo().SerializeJSON(manipulatorValue, _doc.GetAllocator());
                            manipulatorsValue.PushBack(manipulatorValue, _doc.GetAllocator());
                        }
                        objectValue.AddMember("manipulators", manipulatorsValue, _doc.GetAllocator());
                    }

                    if (probot->GetAttachedSensors().size() > 0) {
                        rapidjson::Value attachedSensorsValue;
                        attachedSensorsValue.SetArray();
                        FOREACHC(itattachedsensor, probot->GetAttachedSensors()) {
                            rapidjson::Value attachedSensorValue;
                            (*itattachedsensor)->UpdateAndGetInfo().SerializeJSON(attachedSensorValue, _doc.GetAllocator());
                            attachedSensorsValue.PushBack(attachedSensorValue, _doc.GetAllocator());
                        }
                        objectValue.AddMember("attachedSensors", attachedSensorsValue, _doc.GetAllocator());
                    }

                    if (probot->GetConnectedBodies().size() > 0) {
                        rapidjson::Value connectedBodiesValue;
                        connectedBodiesValue.SetArray();
                        FOREACHC(itconnectedbody, probot->GetConnectedBodies()) {
                            rapidjson::Value connectedBodyValue;
                            (*itconnectedbody)->GetInfo().SerializeJSON(connectedBodyValue, _doc.GetAllocator());
                            connectedBodiesValue.PushBack(connectedBodyValue, _doc.GetAllocator());
                        }
                        objectValue.AddMember("connectedBodies", connectedBodiesValue, _doc.GetAllocator());
                    }

                    SetJsonValueByKey(objectValue, "isRobot", true, _doc.GetAllocator());
                }

                objectsValue.PushBack(objectValue, _doc.GetAllocator());
            }

            if (bodiesValue.MemberCount() > 0) {
                _doc.AddMember("bodies", bodiesValue, _doc.GetAllocator());
            }
            if (objectsValue.MemberCount() > 0) {
                _doc.AddMember("objects", objectsValue, _doc.GetAllocator());
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

    rapidjson::Document& _doc;
};

void RaveWriteJSONFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts)
{
    std::ofstream ofstream(filename.c_str());
    rapidjson::OStreamWrapper ostreamwrapper(ofstream);
    rapidjson::Writer<rapidjson::OStreamWrapper> writer(ostreamwrapper);
    rapidjson::Document doc;

    JSONWriter jsonwriter(atts, doc);
    jsonwriter.Write(penv);
    doc.Accept(writer);
}

void RaveWriteJSONFile(KinBodyPtr pbody, const std::string& filename, const AttributesList& atts)
{
    std::ofstream ofstream(filename.c_str());
    rapidjson::OStreamWrapper ostreamwrapper(ofstream);
    rapidjson::Writer<rapidjson::OStreamWrapper> writer(ostreamwrapper);
    rapidjson::Document doc;

    JSONWriter jsonwriter(atts, doc);
    jsonwriter.Write(pbody);
    doc.Accept(writer);
}

void RaveWriteJSONFile(const std::list<KinBodyPtr>& listbodies, const std::string& filename, const AttributesList& atts)
{
    std::ofstream ofstream(filename.c_str());
    rapidjson::OStreamWrapper ostreamwrapper(ofstream);
    rapidjson::Writer<rapidjson::OStreamWrapper> writer(ostreamwrapper);
    rapidjson::Document doc;

    JSONWriter jsonwriter(atts, doc);
    jsonwriter.Write(listbodies);
    doc.Accept(writer);
}
void RaveWriteJSONStream(EnvironmentBasePtr penv, ostream& os, const AttributesList& atts)
{
    rapidjson::OStreamWrapper ostreamwrapper(os);
    rapidjson::Writer<rapidjson::OStreamWrapper> writer(ostreamwrapper);
    rapidjson::Document doc;

    JSONWriter jsonwriter(atts, doc);
    jsonwriter.Write(penv);
    doc.Accept(writer);
}
void RaveWriteJSONStream(KinBodyPtr pbody, ostream& os, const AttributesList& atts)
{
    rapidjson::OStreamWrapper ostreamwrapper(os);
    rapidjson::Writer<rapidjson::OStreamWrapper> writer(ostreamwrapper);
    rapidjson::Document doc;

    JSONWriter jsonwriter(atts, doc);
    jsonwriter.Write(pbody);
    doc.Accept(writer);
}
void RaveWriteJSONFile(const std::list<KinBodyPtr>& listbodies, ostream& os, const AttributesList& atts)
{
    rapidjson::OStreamWrapper ostreamwrapper(os);
    rapidjson::Writer<rapidjson::OStreamWrapper> writer(ostreamwrapper);
    rapidjson::Document doc;

    JSONWriter jsonwriter(atts, doc);
    jsonwriter.Write(listbodies);
    doc.Accept(writer);
}
}