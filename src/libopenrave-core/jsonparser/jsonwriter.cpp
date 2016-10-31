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
        if (listbodies.size() > 0) {
            EnvironmentBaseConstPtr penv = listbodies.front()->GetEnv();
            RAVE_SERIALIZEJSON_ADDMEMBER(_doc, _doc.GetAllocator(), "unit", penv->GetUnit());

            rapidjson::Value bodiesValue;
            bodiesValue.SetArray();

            rapidjson::Value objectsValue;
            objectsValue.SetArray();

            FOREACHC (it,listbodies) {
                rapidjson::Value bodyValue;
                (*it)->SerializeJSON(bodyValue, _doc.GetAllocator(), SO_DynamicProperties);
                if (!_CheckForExternalWrite(*it))
                {
                    rapidjson::Value objectValue;
                    (*it)->SerializeJSON(objectValue, _doc.GetAllocator(), SO_StaticProperties);

                    // get the id of the object and set it correctly in the instobject
                    std::string id;
                    RAVE_DESERIALIZEJSON_REQUIRED(objectValue, "id", id);

                    if (bodyValue.HasMember("uri")) {
                        bodyValue.RemoveMember("uri");
                    }
                    RAVE_SERIALIZEJSON_ADDMEMBER(bodyValue, _doc.GetAllocator(), "uri", std::string("#") + id);

                    objectsValue.PushBack(objectValue, _doc.GetAllocator());
                }
                else
                {
                    // for external references, need to canonicalize the uri
                    std::string uri = _CanonicalizeURI((*it)->GetURI());
                    if (bodyValue.HasMember("uri")) {
                        bodyValue.RemoveMember("uri");
                    }
                    RAVE_SERIALIZEJSON_ADDMEMBER(bodyValue, _doc.GetAllocator(), "uri", uri);
                }

                bodiesValue.PushBack(bodyValue, _doc.GetAllocator());
            }

            _doc.AddMember("bodies", bodiesValue, _doc.GetAllocator());
            _doc.AddMember("objects", objectsValue, _doc.GetAllocator());
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
            path = path.substr(colonindex + 2);
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

        std::string newuri = scheme + path;
        if (fragment.size() > 0) {
            newuri += "#" + fragment;
        }
        return newuri;
    }

    std::string _vForceResolveOpenRAVEScheme; ///< if specified, writer will attempt to convert a local system URI (**file:/**) to a a relative path with respect to $OPENRAVE_DATA paths and use **customscheme** as the scheme
    std::list<std::string> _listExternalRefExports; ///< body names to try to export externally

    bool _bExternalRefAllBodies; ///< if true, attempts to externally write all bodies
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

}