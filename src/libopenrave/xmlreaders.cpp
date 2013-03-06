// -*- coding: utf-8 -*-
// Copyright (C) 2006-2013 Rosen Diankov <rosen.diankov@gmail.com>
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
#include <openrave/xmlreaders.h>

#include <boost/lexical_cast.hpp>

namespace OpenRAVE {
namespace xmlreaders {

StringXMLReadable::StringXMLReadable(const std::string& xmlid, const std::string& data) : XMLReadable(xmlid), _data(data)
{
}

void StringXMLReadable::Serialize(BaseXMLWriterPtr writer, int options) const
{
    if( writer->GetFormat() == "collada" ) {
        AttributesList atts;
        atts.push_back(make_pair(string("type"),"stringxmlreadable"));
        atts.push_back(make_pair(string("name"), GetXMLId()));
        BaseXMLWriterPtr child = writer->AddChild("extra",atts);
        atts.clear();
        atts.push_back(make_pair(string("profile"), string("OpenRAVE")));
        writer = child->AddChild("technique",atts)->AddChild("data");
    }

    writer->SetCharData(_data);
}

const std::string& StringXMLReadable::GetData() const
{
    return _data;
}

HierarchicalXMLReadable::HierarchicalXMLReadable(const std::string& xmlid, const AttributesList& atts) : XMLReadable(xmlid), _atts(atts)
{
}

void HierarchicalXMLReadable::Serialize(BaseXMLWriterPtr writer, int options) const
{
    writer->SetCharData(_data);
    for(std::list<HierarchicalXMLReadablePtr>::const_iterator it = _listchildren.begin(); it != _listchildren.end(); ++it) {
        BaseXMLWriterPtr childwriter = writer->AddChild((*it)->GetXMLId(), (*it)->_atts);
        (*it)->Serialize(childwriter,options);
    }
}

TrajectoryReader::TrajectoryReader(EnvironmentBasePtr penv, TrajectoryBasePtr ptraj, const AttributesList& atts) : _ptraj(ptraj)
{
    _bInReadable = false;
    _datacount = 0;
    FOREACHC(itatt, atts) {
        if( itatt->first == "type" ) {
            if( !!_ptraj ) {
                OPENRAVE_ASSERT_OP(_ptraj->GetXMLId(),==,itatt->second );
            }
            else {
                _ptraj = RaveCreateTrajectory(penv, itatt->second);
            }
        }
    }
    if( !_ptraj ) {
        _ptraj = RaveCreateTrajectory(penv, "");
    }
}

BaseXMLReader::ProcessElement TrajectoryReader::startElement(const std::string& name, const AttributesList& atts)
{
    _ss.str("");
    if( !!_pcurreader ) {
        if( _pcurreader->startElement(name, atts) == PE_Support ) {
            return PE_Support;
        }
        return PE_Ignore;
    }
    if( _bInReadable ) {
        _pcurreader.reset(new HierarchicalXMLReader(name,atts));
        return PE_Support;
    }
    if( name == "trajectory" ) {
        _pcurreader.reset(new TrajectoryReader(_ptraj->GetEnv(), _ptraj, atts));
        return PE_Support;
    }
    else if( name == "configuration" ) {
        _pcurreader.reset(new ConfigurationSpecification::Reader(_spec));
        return PE_Support;
    }
    else if( name == "readable" ) {
        _bInReadable = true;
        return PE_Support;
    }
    else if( name == "data" ) {
        _vdata.resize(0);
        _datacount = 0;
        FOREACHC(itatt,atts) {
            if( itatt->first == "count" ) {
                _datacount = boost::lexical_cast<int>(itatt->second);
            }
        }
        return PE_Support;
    }
    else if( name == "description" ) {
        return PE_Support;
    }
    return PE_Pass;
}

bool TrajectoryReader::endElement(const std::string& name)
{
    if( !!_pcurreader ) {
        if( _pcurreader->endElement(name) ) {
            if( _bInReadable ) {
                HierarchicalXMLReaderPtr reader = boost::dynamic_pointer_cast<HierarchicalXMLReader>(_pcurreader);
                _ptraj->SetReadableInterface(reader->GetReadable()->GetXMLId(), reader->GetReadable());
                _pcurreader.reset();
            }
            else {
                if( !!boost::dynamic_pointer_cast<ConfigurationSpecification::Reader>(_pcurreader) ) {
                    BOOST_ASSERT(_spec.IsValid());
                    _ptraj->Init(_spec);
                }
                bool bret = !!boost::dynamic_pointer_cast<TrajectoryReader>(_pcurreader);
                _pcurreader.reset();
                if( bret ) {
                    return true;
                }
            }
        }
    }
    else if( name == "data" ) {
        _vdata.resize(_spec.GetDOF()*_datacount);
        for(size_t i = 0; i < _vdata.size(); ++i) {
            _ss >> _vdata[i];
        }
        if( !_ss ) {
            RAVELOG_WARN("failed treading trajectory <data>\n");
        }
        else {
            _ptraj->Insert(_ptraj->GetNumWaypoints(),_vdata);
        }
    }
    else if( name == "description" ) {
        _ptraj->SetDescription(_ss.str());
    }
    else if( name == "trajectory" ) {
        return true;
    }
    else if( name == "readable" ) {
        _bInReadable = false;
    }
    return false;
}

void TrajectoryReader::characters(const std::string& ch)
{
    if( !!_pcurreader ) {
        _pcurreader->characters(ch);
    }
    else {
        _ss.clear();
        _ss << ch;
    }
}

GeometryInfoReader::GeometryInfoReader(KinBody::GeometryInfoPtr pgeom, const AttributesList& atts) : _pgeom(pgeom)
{
    _bOverwriteDiffuse = _bOverwriteAmbient = _bOverwriteTransparency = false;
    string type;
    bool bVisible = true, bModifiable = true;
    FOREACHC(itatt,atts) {
        if( itatt->first == "type") {
            type = itatt->second;
        }
        else if( itatt->first == "render" ) {
            // set draw to false only if atts[i]==false
            bVisible = _stricmp(itatt->second.c_str(), "false")!=0 && itatt->second!="0";
        }
        else if( itatt->first == "modifiable" ) {
            bModifiable = !(_stricmp(itatt->second.c_str(), "false") == 0 || itatt->second=="0");
        }
    }

    if( type.size() == 0 ) {
        RAVELOG_INFOA("no geometry type, defaulting to box\n");
        type = "box";
    }

    _pgeom.reset(new KinBody::GeometryInfo());
    if( _stricmp(type.c_str(), "box") == 0 ) {
        _pgeom->_type = GT_Box;
    }
    else if( _stricmp(type.c_str(), "sphere") == 0 ) {
        _pgeom->_type = GT_Sphere;
    }
    else if( _stricmp(type.c_str(), "cylinder") == 0 ) {
        _pgeom->_type = GT_Cylinder;
    }
    else if( _stricmp(type.c_str(), "trimesh") == 0 ) {
        _pgeom->_type = GT_TriMesh;
    }
    else {
        RAVELOG_WARN(str(boost::format("type %s not supported\n")%type));
    }
    _pgeom->_bVisible = bVisible;
    _pgeom->_bModifiable = bModifiable;
}

BaseXMLReader::ProcessElement GeometryInfoReader::startElement(const std::string& xmlname, const AttributesList& atts)
{
    _ss.str("");
    if( !!_pcurreader ) {
        if( _pcurreader->startElement(xmlname, atts) == PE_Support ) {
            return PE_Support;
        }
        return PE_Ignore;
    }

    if( xmlname == _pgeom->GetXMLId() || xmlname == "geom" ) {
        _pcurreader.reset(new GeometryInfoReader(_pgeom, atts));
        return PE_Support;
    }
    if( xmlname == "render" ) {
        // check the attributes first
        FOREACHC(itatt,atts) {
            if( itatt->first == "file" ) {
                _pgeom->_filenamerender = itatt->second;
            }
            else if( itatt->first == "scale" ) {
                _pgeom->_vRenderScale = Vector(1,1,1);
                stringstream sslocal(itatt->second);
                sslocal >> _pgeom->_vRenderScale.x; _pgeom->_vRenderScale.y = _pgeom->_vRenderScale.z = _pgeom->_vRenderScale.x;
                sslocal >> _pgeom->_vRenderScale.y >> _pgeom->_vRenderScale.z;
            }
        }
    }
    else if( xmlname == "collision" ) {
        // check the attributes first
        FOREACHC(itatt,atts) {
            if( itatt->first == "file" ) {
                _pgeom->_filenamecollision = itatt->second;
            }
            else if( itatt->first == "scale" ) {
                _pgeom->_vCollisionScale = Vector(1,1,1);
                stringstream sslocal(itatt->second);
                sslocal >> _pgeom->_vCollisionScale.x; _pgeom->_vCollisionScale.y = _pgeom->_vCollisionScale.z = _pgeom->_vCollisionScale.x;
                sslocal >> _pgeom->_vCollisionScale.y >> _pgeom->_vCollisionScale.z;
            }
        }
    }

    static boost::array<string,11> tags = { { "translation", "rotationmat", "rotationaxis", "quat", "diffusecolor", "ambientcolor", "transparency", "render", "extents", "radius", "height"}};
    if( find(tags.begin(),tags.end(),xmlname) != tags.end() ) {
        return PE_Support;
    }
    switch(_pgeom->_type) {
    case GT_TriMesh:
        if(xmlname=="collision"|| xmlname=="data" || xmlname=="vertices" ) {
            return PE_Support;
        }
        break;
    default:
        break;
    }
    return PE_Pass;
}

bool GeometryInfoReader::endElement(const std::string& xmlname)
{
    if( !!_pcurreader ) {
        if( _pcurreader->endElement(xmlname) ) {
            bool bret = !!boost::dynamic_pointer_cast<GeometryInfoReader>(_pcurreader);
            _pcurreader.reset();
            if( bret ) {
                return true;
            }
        }
        return false;
    }

    if( xmlname == _pgeom->GetXMLId() || xmlname == "geom" ) {
        return true;
    }
    else if( xmlname == "translation" ) {
        Vector v;
        _ss >>v.x >> v.y >> v.z;
        _pgeom->_t.trans += v;
    }
    else if( xmlname == "rotationmat" ) {
        TransformMatrix tnew;
        _ss >> tnew.m[0] >> tnew.m[1] >> tnew.m[2] >> tnew.m[4] >> tnew.m[5] >> tnew.m[6] >> tnew.m[8] >> tnew.m[9] >> tnew.m[10];
        _pgeom->_t.rot = (Transform(tnew)*_pgeom->_t).rot;
    }
    else if( xmlname == "rotationaxis" ) {
        Vector vaxis; dReal fangle=0;
        _ss >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
        Transform tnew; tnew.rot = quatFromAxisAngle(vaxis, fangle * PI / 180.0f);
        _pgeom->_t.rot = (tnew*_pgeom->_t).rot;
    }
    else if( xmlname == "quat" ) {
        Transform tnew;
        _ss >> tnew.rot.x >> tnew.rot.y >> tnew.rot.z >> tnew.rot.w;
        tnew.rot.normalize4();
        _pgeom->_t.rot = (tnew*_pgeom->_t).rot;
    }
    else if( xmlname == "render" ) {
        if( _pgeom->_filenamerender.size() == 0 ) {
            _pgeom->_vRenderScale = Vector(1,1,1);
            _ss >> _pgeom->_filenamerender;
            _ss >> _pgeom->_vRenderScale.x; _pgeom->_vRenderScale.y = _pgeom->_vRenderScale.z = _pgeom->_vRenderScale.x;
            _ss >> _pgeom->_vRenderScale.y >> _pgeom->_vRenderScale.z;
        }
    }
    else if( xmlname == "diffusecolor" ) {
        _bOverwriteDiffuse = true;
        _ss >> _pgeom->_vDiffuseColor.x >> _pgeom->_vDiffuseColor.y >> _pgeom->_vDiffuseColor.z;
    }
    else if( xmlname == "ambientcolor" ) {
        _bOverwriteAmbient = true;
        _ss >> _pgeom->_vAmbientColor.x >> _pgeom->_vAmbientColor.y >> _pgeom->_vAmbientColor.z;
    }
    else if( xmlname == "transparency" ) {
        _bOverwriteTransparency = true;
        _ss >> _pgeom->_fTransparency;
    }
    else {
        // could be type specific features
        switch(_pgeom->_type) {
        case GT_Sphere:
            if( xmlname == "radius" ) {
                _ss >> _pgeom->_vGeomData.x;
            }
            break;
        case GT_Box:
            if( xmlname == "extents" ) {
                _ss >> _pgeom->_vGeomData.x >> _pgeom->_vGeomData.y >> _pgeom->_vGeomData.z;
            }
            break;
        case GT_Cylinder:
            if( xmlname == "radius") {
                _ss >> _pgeom->_vGeomData.x;
            }
            else if( xmlname == "height" ) {
                _ss >> _pgeom->_vGeomData.y;
            }
            break;
        case GT_TriMesh:
            if(( xmlname == "data") ||( xmlname == "collision") ) {
                if( _pgeom->_filenamecollision.size() == 0 ) {
                    // check the attributes first
                    _pgeom->_vCollisionScale = Vector(1,1,1);
                    _ss >> _pgeom->_filenamecollision;
                    _ss >> _pgeom->_vCollisionScale.x; _pgeom->_vCollisionScale.y = _pgeom->_vCollisionScale.z = _pgeom->_vCollisionScale.x;
                    _ss >> _pgeom->_vCollisionScale.y >> _pgeom->_vCollisionScale.z;
                }
            }
            else if( xmlname == "vertices" ) {
                vector<dReal> values((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
                if( (values.size()%9) ) {
                    RAVELOG_WARN(str(boost::format("number of points specified in the vertices field needs to be a multiple of 3 (it is %d), ignoring...\n")%values.size()));
                }
                else {
                    _pgeom->_meshcollision.vertices.resize(values.size()/3);
                    _pgeom->_meshcollision.indices.resize(values.size()/3);
                    vector<dReal>::iterator itvalue = values.begin();
                    size_t i = 0;
                    FOREACH(itv,_pgeom->_meshcollision.vertices) {
                        itv->x = *itvalue++;
                        itv->y = *itvalue++;
                        itv->z = *itvalue++;
                        _pgeom->_meshcollision.indices[i] = i;
                        ++i;
                    }
                }
            }
            break;
        default:
            _pcurreader.reset(new DummyXMLReader(xmlname,"geom"));
        }
    }


    return false;
}

void GeometryInfoReader::characters(const std::string& ch)
{
    if( !!_pcurreader ) {
        _pcurreader->characters(ch);
    }
    else {
        _ss.clear();
        _ss << ch;
    }
}

HierarchicalXMLReader::HierarchicalXMLReader(const std::string& xmlid, const AttributesList& atts) : _xmlid(xmlid)
{
    _readable.reset(new HierarchicalXMLReadable(xmlid,atts));
}

BaseXMLReader::ProcessElement HierarchicalXMLReader::startElement(const std::string& name, const AttributesList& atts)
{
    if( !!_pcurreader ) {
        if( _pcurreader->startElement(name, atts) == PE_Support ) {
            return PE_Support;
        }
        return PE_Ignore;
    }

    _pcurreader.reset(new HierarchicalXMLReader(name,atts));
    return PE_Support;
}

bool HierarchicalXMLReader::endElement(const std::string& name)
{
    if( !!_pcurreader ) {
        if( _pcurreader->endElement(name) ) {
            _readable->_listchildren.push_back(_pcurreader->_readable);
            _pcurreader.reset();
        }
        return false;
    }

    if( name == _xmlid ) {
        return true;
    }
    RAVELOG_ERROR(str(boost::format("invalid xml tag %s, expected %s\n")%name%_xmlid));
    return false;
}

void HierarchicalXMLReader::characters(const std::string& ch)
{
    if( !_pcurreader ) {
        _readable->_data += ch;
    }
    else {
        _pcurreader->characters(ch);
    }
}

XMLReadablePtr HierarchicalXMLReader::GetReadable()
{
    return _readable;
}


StreamXMLWriter::StreamXMLWriter(const std::string& xmltag, const AttributesList& atts) : _xmltag(xmltag), _atts(atts)
{
}

const std::string& StreamXMLWriter::GetFormat() const
{
    static const std::string format("xml");
    return format;
}

void StreamXMLWriter::SetCharData(const std::string& data)
{
    _data = data;
}

BaseXMLWriterPtr StreamXMLWriter::AddChild(const std::string& xmltag, const AttributesList& atts)
{
    boost::shared_ptr<StreamXMLWriter> child(new StreamXMLWriter(xmltag,atts));
    _listchildren.push_back(child);
    return child;
}

void StreamXMLWriter::Serialize(std::ostream& stream)
{
    if( _xmltag.size() > 0 ) {
        stream << "<" << _xmltag << " ";
        FOREACHC(it, _atts) {
            stream << it->first << "=\"" << it->second << "\" ";
        }
        // don't skip any lines since could affect reading back _data
        stream << ">";
    }
    if( _data.size() > 0 ) {
        if( _xmltag.size() > 0 ) {
            stream << "<![CDATA[" << _data << "]]>";
        }
        else {
            // there's no tag, so can render plaintext
            stream << _data;
        }
    }
    FOREACHC(it, _listchildren) {
        (*it)->Serialize(stream);
    }
    if( _xmltag.size() > 0 ) {
        stream << "</" << _xmltag << ">" << std::endl;
    }
}


} // xmlreaders
} // OpenRAVE
