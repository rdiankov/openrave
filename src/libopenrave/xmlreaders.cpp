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

#define LIBXML_SAX1_ENABLED
#include <libxml/globals.h>
#include <libxml/xmlerror.h>
#include <libxml/parser.h>
#include <libxml/parserInternals.h> // only for xmlNewInputFromFile()
#include <libxml/tree.h>

#include <libxml/debugXML.h>
#include <libxml/xmlmemory.h>

namespace OpenRAVE {

namespace xmlreaders {

HierarchicalXMLReadable::HierarchicalXMLReadable(const std::string& xmlid, const AttributesList& atts) : Readable(xmlid), _atts(atts)
{
}

bool HierarchicalXMLReadable::SerializeXML(BaseXMLWriterPtr writer, int options) const
{
    writer->SetCharData(_data);
    for(std::list<HierarchicalXMLReadablePtr>::const_iterator it = _listchildren.begin(); it != _listchildren.end(); ++it) {
        BaseXMLWriterPtr childwriter = writer->AddChild((*it)->GetXMLId(), (*it)->_atts);
        (*it)->SerializeXML(childwriter,options);
    }
    return true;
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
            throw OPENRAVE_EXCEPTION_FORMAT(_("failed reading %d numbers from trajectory <data> element"), _vdata.size(), ORE_Assert);
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
    _sGroupName = "self";
    string type, name;
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
        else if( itatt->first == "group" && !itatt->second.empty() ) {
            _sGroupName = itatt->second;
        }
        else if( itatt->first == "name" && !itatt->second.empty() ) {
            name = itatt->second;
        }
    }

    if( type.size() == 0 ) {
        RAVELOG_INFOA("no geometry type, defaulting to box\n");
        type = "box";
    }

    _pgeom.reset(new KinBody::GeometryInfo());
    _pgeom->_name = name;
    if( _stricmp(type.c_str(), "none") == 0 ) {
        _pgeom->_type = GT_None;
    }
    else if( _stricmp(type.c_str(), "box") == 0 ) {
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
    else if( _stricmp(type.c_str(), "container") == 0 ) {
        _pgeom->_type = GT_Container;
    }
    else if( _stricmp(type.c_str(), "cage") == 0 ) {
        _pgeom->_type = GT_Cage;
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

    if( xmlname == "geometry" || xmlname == "geom" ) {
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

    static boost::array<string,14> tags = { { "translation", "rotationmat", "rotationaxis", "quat", "diffusecolor", "ambientcolor", "transparency", "render", "extents", "halfextents", "fullextents", "radius", "height", "name"}};
    if( find(tags.begin(),tags.end(),xmlname) != tags.end() ) {
        return PE_Support;
    }
    switch(_pgeom->_type) {
    case GT_Container:
        if( xmlname == "outer_extents" || xmlname == "inner_extents" || xmlname == "bottom_cross" || xmlname == "bottom" ) {
            return PE_Support;
        }
        break;
    case GT_Cage:
        if( xmlname == "sidewall" || xmlname == "transf" || xmlname == "vExtents" || xmlname == "type" ) {
            return PE_Support;
        }
        break;
    case GT_TriMesh:
        if( xmlname == "collision" || xmlname == "data" || xmlname == "vertices" ) {
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

    if( xmlname == "geometry" || xmlname == "geom" ) {
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
    else if( xmlname == "name" ) {
        _ss >> _pgeom->_name;
    }
    else {
        // could be type specific features
        switch(_pgeom->_type) {
        case GT_None:
            // Do nothing
            break;

        case GT_Sphere:
            if( xmlname == "radius" ) {
                _ss >> _pgeom->_vGeomData.x;
            }
            break;
        case GT_Box:
            if( xmlname == "extents" || xmlname == "halfextents" ) {
                _ss >> _pgeom->_vGeomData.x >> _pgeom->_vGeomData.y >> _pgeom->_vGeomData.z;
            }
            else if( xmlname == "fullextents" ) {
                _ss >> _pgeom->_vGeomData.x >> _pgeom->_vGeomData.y >> _pgeom->_vGeomData.z;
                _pgeom->_vGeomData *= 0.5;
            }

            break;
        case GT_Container:
            if( xmlname == "outer_extents" ) {
                _ss >> _pgeom->_vGeomData.x >> _pgeom->_vGeomData.y >> _pgeom->_vGeomData.z;
            }
            if( xmlname == "inner_extents" ) {
                _ss >> _pgeom->_vGeomData2.x >> _pgeom->_vGeomData2.y >> _pgeom->_vGeomData2.z;
            }
            if( xmlname == "bottom_cross" ) {
                _ss >> _pgeom->_vGeomData3.x >> _pgeom->_vGeomData3.y >> _pgeom->_vGeomData3.z;
            }
            if( xmlname == "bottom" ) {
                _ss >> _pgeom->_vGeomData4.x >> _pgeom->_vGeomData4.y >> _pgeom->_vGeomData4.z;
            }

            break;
        case GT_Cage:
            if( xmlname == "sidewall" ) {
                _pgeom->_vSideWalls.push_back({});
            }
            if( xmlname == "transf" ) {
                _ss >> _pgeom->_vSideWalls.back().transf;
            }
            if( xmlname == "vExtents" ) {
                _ss >> _pgeom->_vSideWalls.back().vExtents;
            }
            if( xmlname == "type" ) {
                int32_t type;
                _ss >> type;
                _pgeom->_vSideWalls.back().type = static_cast<KinBody::GeometryInfo::SideWallType>(type);
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


ElectricMotorActuatorInfoReader::ElectricMotorActuatorInfoReader(ElectricMotorActuatorInfoPtr pinfo, const AttributesList& atts) : _pinfo(pinfo)
{
    string type;
    FOREACHC(itatt,atts) {
        if( itatt->first == "type") {
            type = itatt->second;
        }
    }

    if( type.size() == 0 ) {
        RAVELOG_INFOA("no actuator type, defaulting to electric_motor\n");
        type = "electric_motor";
    }

    if( type != "electric_motor" ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("does not support actuator '%s' type"), type, ORE_InvalidArguments);
    }

    _pinfo.reset(new ElectricMotorActuatorInfo());
}

BaseXMLReader::ProcessElement ElectricMotorActuatorInfoReader::startElement(const std::string& xmlname, const AttributesList& atts)
{
    _ss.str("");
    if( !!_pcurreader ) {
        if( _pcurreader->startElement(xmlname, atts) == PE_Support ) {
            return PE_Support;
        }
        return PE_Ignore;
    }

    if( xmlname == "actuator" ) {
        _pcurreader.reset(new ElectricMotorActuatorInfoReader(_pinfo, atts));
        return PE_Support;
    }

    static boost::array<string, 18> tags = { { "gear_ratio", "assigned_power_rating", "max_speed", "no_load_speed", "stall_torque", "nominal_speed_torque_point", "max_speed_torque_point", "nominal_torque", "rotor_inertia", "torque_constant", "nominal_voltage", "speed_constant", "starting_current", "terminal_resistance", "coloumb_friction", "viscous_friction", "model_type", "max_instantaneous_torque", } };
    if( find(tags.begin(),tags.end(),xmlname) != tags.end() ) {
        return PE_Support;
    }
    return PE_Pass;
}

bool ElectricMotorActuatorInfoReader::endElement(const std::string& xmlname)
{
    if( !!_pcurreader ) {
        if( _pcurreader->endElement(xmlname) ) {
            bool bret = !!boost::dynamic_pointer_cast<ElectricMotorActuatorInfoReader>(_pcurreader);
            _pcurreader.reset();
            if( bret ) {
                return true;
            }
        }
        return false;
    }

    if( xmlname == "actuator" ) {
        return true;
    }
    else if( xmlname == "model_type" ) {
        _ss >> _pinfo->model_type;
    }
    else if( xmlname == "assigned_power_rating" ) {
        _ss >> _pinfo->assigned_power_rating;
    }
    else if( xmlname == "max_speed" ) {
        _ss >> _pinfo->max_speed;
    }
    else if( xmlname == "no_load_speed" ) {
        _ss >> _pinfo->no_load_speed;
    }
    else if( xmlname == "stall_torque" ) {
        _ss >> _pinfo->stall_torque;
    }
    else if( xmlname == "max_instantaneous_torque" ) {
        _ss >> _pinfo->max_instantaneous_torque;
    }
    else if( xmlname == "nominal_speed_torque_point" ) {
        dReal speed=0, torque=0;
        _ss >> speed >> torque;
        // should be from increasing speed.
        size_t insertindex = 0;
        while(insertindex < _pinfo->nominal_speed_torque_points.size()) {
            if( speed < _pinfo->nominal_speed_torque_points.at(insertindex).first ) {
                break;
            }
            ++insertindex;
        }
        _pinfo->nominal_speed_torque_points.insert(_pinfo->nominal_speed_torque_points.begin()+insertindex, make_pair(speed,torque));
    }
    else if( xmlname == "max_speed_torque_point" ) {
        dReal speed=0, torque=0;
        _ss >> speed >> torque;
        // should be from increasing speed.
        size_t insertindex = 0;
        while(insertindex < _pinfo->max_speed_torque_points.size()) {
            if( speed < _pinfo->max_speed_torque_points.at(insertindex).first ) {
                break;
            }
            ++insertindex;
        }
        _pinfo->max_speed_torque_points.insert(_pinfo->max_speed_torque_points.begin()+insertindex, make_pair(speed,torque));
    }
    else if( xmlname == "nominal_torque" ) {
        _ss >> _pinfo->nominal_torque;
    }
    else if( xmlname == "rotor_inertia" ) {
        _ss >> _pinfo->rotor_inertia;
    }
    else if( xmlname == "torque_constant" ) {
        _ss >> _pinfo->torque_constant;
    }
    else if( xmlname == "nominal_voltage" ) {
        _ss >> _pinfo->nominal_voltage;
    }
    else if( xmlname == "speed_constant" ) {
        _ss >> _pinfo->speed_constant;
    }
    else if( xmlname == "starting_current" ) {
        _ss >> _pinfo->starting_current;
    }
    else if( xmlname == "terminal_resistance" ) {
        _ss >> _pinfo->terminal_resistance;
    }
    else if( xmlname == "gear_ratio" ) {
        _ss >> _pinfo->gear_ratio;
    }
    else if( xmlname == "coloumb_friction" ) {
        _ss >> _pinfo->coloumb_friction;
    }
    else if( xmlname == "viscous_friction" ) {
        _ss >> _pinfo->viscous_friction;
    }
    else {
        RAVELOG_WARN_FORMAT("could not process tag %s", xmlname);
    }

    return false;
}

void ElectricMotorActuatorInfoReader::characters(const std::string& ch)
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

ReadablePtr HierarchicalXMLReader::GetReadable()
{
    return _readable;
}

HierarchicalXMLReadablePtr HierarchicalXMLReader::GetHierarchicalReadable()
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


namespace LocalXML {

void RaveXMLErrorFunc(void *ctx, const char *msg, ...)
{
    va_list args;

    va_start(args, msg);
    RAVELOG_ERROR("XML Parse error: ");
    vprintf(msg,args);
    va_end(args);
}

struct XMLREADERDATA
{
    XMLREADERDATA(BaseXMLReader& reader, xmlParserCtxtPtr ctxt) : _reader(reader), _ctxt(ctxt) {
    }
    BaseXMLReader& _reader;
    BaseXMLReaderPtr _pdummy;
    xmlParserCtxtPtr _ctxt;
};

void DefaultStartElementSAXFunc(void *ctx, const xmlChar *name, const xmlChar **atts)
{
    AttributesList listatts;
    if( atts != NULL ) {
        for (int i = 0; (atts[i] != NULL); i+=2) {
            listatts.emplace_back((const char*)atts[i], (const char*)atts[i+1]);
            std::transform(listatts.back().first.begin(), listatts.back().first.end(), listatts.back().first.begin(), ::tolower);
        }
    }

    XMLREADERDATA* pdata = (XMLREADERDATA*)ctx;
    string s = (const char*)name;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    if( !!pdata->_pdummy ) {
        RAVELOG_VERBOSE(str(boost::format("unknown field %s\n")%s));
        pdata->_pdummy->startElement(s,listatts);
    }
    else {
        BaseXMLReader::ProcessElement pestatus = pdata->_reader.startElement(s, listatts);
        if( pestatus != BaseXMLReader::PE_Support ) {
            // not handling, so create a temporary class to handle it
            pdata->_pdummy.reset(new DummyXMLReader(s,"(libxml)"));
        }
    }
}

void DefaultEndElementSAXFunc(void *ctx, const xmlChar *name)
{
    XMLREADERDATA* pdata = (XMLREADERDATA*)ctx;
    string s = (const char*)name;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    if( !!pdata->_pdummy ) {
        if( pdata->_pdummy->endElement(s) ) {
            pdata->_pdummy.reset();
        }
    }
    else {
        if( pdata->_reader.endElement(s) ) {
            //RAVEPRINT(L"%s size read %d\n", name, data->_ctxt->input->consumed);
            xmlStopParser(pdata->_ctxt);
        }
    }
}

void DefaultCharactersSAXFunc(void *ctx, const xmlChar *ch, int len)
{
    XMLREADERDATA* pdata = (XMLREADERDATA*)ctx;
    if( !!pdata->_pdummy ) {
        pdata->_pdummy->characters(string((const char*)ch, len));
    }
    else {
        pdata->_reader.characters(string((const char*)ch, len));
    }
}

bool xmlDetectSAX2(xmlParserCtxtPtr ctxt)
{
    if (ctxt == NULL) {
        return false;
    }
#ifdef LIBXML_SAX1_ENABLED
    if (ctxt->sax &&  ctxt->sax->initialized == XML_SAX2_MAGIC && (ctxt->sax->startElementNs != NULL || ctxt->sax->endElementNs != NULL)) {
        ctxt->sax2 = 1;
    }
#else
    ctxt->sax2 = 1;
#endif

    ctxt->str_xml = xmlDictLookup(ctxt->dict, BAD_CAST "xml", 3);
    ctxt->str_xmlns = xmlDictLookup(ctxt->dict, BAD_CAST "xmlns", 5);
    ctxt->str_xml_ns = xmlDictLookup(ctxt->dict, XML_XML_NAMESPACE, 36);
    if ( ctxt->str_xml==NULL || ctxt->str_xmlns==NULL || ctxt->str_xml_ns == NULL) {
        return false;
    }
    return true;
}

} // end namespace LocalXML

bool ParseXMLData(BaseXMLReader& reader, const char* buffer, int size)
{
    static xmlSAXHandler s_DefaultSAXHandler = { 0};
    if( size <= 0 ) {
        size = strlen(buffer);
    }
    if( !s_DefaultSAXHandler.initialized ) {
        // first time, so init
        s_DefaultSAXHandler.startElement = LocalXML::DefaultStartElementSAXFunc;
        s_DefaultSAXHandler.endElement = LocalXML::DefaultEndElementSAXFunc;
        s_DefaultSAXHandler.characters = LocalXML::DefaultCharactersSAXFunc;
        s_DefaultSAXHandler.error = LocalXML::RaveXMLErrorFunc;
        s_DefaultSAXHandler.initialized = 1;
    }

    xmlSAXHandlerPtr sax = &s_DefaultSAXHandler;
    int ret = 0;
    xmlParserCtxtPtr ctxt;

    ctxt = xmlCreateMemoryParserCtxt(buffer, size);
    if (ctxt == NULL) {
        return false;
    }
    if (ctxt->sax != (xmlSAXHandlerPtr) &xmlDefaultSAXHandler) {
        xmlFree(ctxt->sax);
    }
    ctxt->sax = sax;
    LocalXML::xmlDetectSAX2(ctxt);

    LocalXML::XMLREADERDATA readerdata(reader, ctxt);
    ctxt->userData = &readerdata;

    xmlParseDocument(ctxt);

    if (ctxt->wellFormed) {
        ret = 0;
    }
    else {
        if (ctxt->errNo != 0) {
            ret = ctxt->errNo;
        }
        else {
            ret = -1;
        }
    }
    if (sax != NULL) {
        ctxt->sax = NULL;
    }
    if (ctxt->myDoc != NULL) {
        xmlFreeDoc(ctxt->myDoc);
        ctxt->myDoc = NULL;
    }
    xmlFreeParserCtxt(ctxt);

    return ret==0;
}

} // xmlreaders
} // OpenRAVE
