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

/** \file xmlreaders.h
    \brief classes for reading common OpenRAVE objects from XML

    This file is optional and not automatically included with openrave.h
 */
#ifndef OPENRAVE_XMLREADERS_H
#define OPENRAVE_XMLREADERS_H

#include <openrave/openrave.h>

namespace OpenRAVE {

namespace xmlreaders {

class OPENRAVE_API StringXMLReadable : public XMLReadable
{
public:
    StringXMLReadable(const std::string& xmlid, const std::string& data);
    virtual void Serialize(BaseXMLWriterPtr writer, int options=0) const;
    const std::string& GetData() const;
    std::string _data;
};

typedef std::shared_ptr<StringXMLReadable> StringXMLReadablePtr;

/// \brief maintains a hierarchy of classes each containing the xml attributes and data
class OPENRAVE_API HierarchicalXMLReadable : public XMLReadable
{
public:
    HierarchicalXMLReadable(const std::string& xmlid, const AttributesList& atts);
    virtual ~HierarchicalXMLReadable() {
    }
    virtual void Serialize(BaseXMLWriterPtr writer, int options=0) const;
    std::string _data;
    AttributesList _atts;
    std::list<std::shared_ptr<HierarchicalXMLReadable> > _listchildren;
};

typedef std::shared_ptr<HierarchicalXMLReadable> HierarchicalXMLReadablePtr;

/// \brief create a xml parser for trajectories
class OPENRAVE_API TrajectoryReader : public BaseXMLReader
{
public:
    /// \param env the environment used to create the trajectory
    /// \param traj can optionally pass a trajectory to initialize if need to read into an existing trajectory, but the pointer can be empty
    /// \param atts attributes passed from <trajectory> tag
    TrajectoryReader(EnvironmentBasePtr env, TrajectoryBasePtr traj = TrajectoryBasePtr(), const AttributesList& atts=AttributesList());
    virtual ProcessElement startElement(const std::string& name, const AttributesList& atts);
    virtual bool endElement(const std::string& name);
    virtual void characters(const std::string& ch);

    inline TrajectoryBasePtr GetTrajectory() const {
        return _ptraj;
    }

protected:
    TrajectoryBasePtr _ptraj;
    std::stringstream _ss;
    ConfigurationSpecification _spec;
    BaseXMLReaderPtr _pcurreader;
    int _datacount;
    std::vector<dReal> _vdata;
    bool _bInReadable;
};

typedef std::shared_ptr<TrajectoryReader> TrajectoryReaderPtr;

/// \brief create a xml parser for \ref KinBody::GeometryInfo
class OPENRAVE_API GeometryInfoReader : public BaseXMLReader
{
public:
    /// \param env the environment used to create the trajectory
    /// \param traj can optionally pass a trajectory to initialize if need to read into an existing trajectory, but the pointer can be empty
    /// \param atts attributes passed from <trajectory> tag
    GeometryInfoReader(KinBody::GeometryInfoPtr geom = KinBody::GeometryInfoPtr(), const AttributesList& atts=AttributesList());
    virtual ProcessElement startElement(const std::string& name, const AttributesList& atts);
    virtual bool endElement(const std::string& name);
    virtual void characters(const std::string& ch);

    inline KinBody::GeometryInfoPtr GetGeometryInfo() const {
        return _pgeom;
    }

    inline bool IsOverwriteDiffuse() const {
        return _bOverwriteDiffuse;
    }
    inline bool IsOverwriteAmbient() const {
        return _bOverwriteAmbient;
    }
    inline bool IsOverwriteTransparency() const {
        return _bOverwriteTransparency;
    }

    inline const std::string &GetGroupName() const {
        return _sGroupName;
    }
protected:
    KinBody::GeometryInfoPtr _pgeom;
    std::stringstream _ss;
    BaseXMLReaderPtr _pcurreader;

    bool _bOverwriteDiffuse, _bOverwriteAmbient, _bOverwriteTransparency;
    std::string _sGroupName;
};

typedef std::shared_ptr<GeometryInfoReader> GeometryInfoReaderPtr;

/// \brief create a xml parser for \ref ElectricMotorActuatorInfo
class OPENRAVE_API ElectricMotorActuatorInfoReader : public BaseXMLReader
{
public:
    /// \param env the environment used to create the trajectory
    /// \param traj can optionally pass a trajectory to initialize if need to read into an existing trajectory, but the pointer can be empty
    /// \param atts attributes passed from <trajectory> tag
    ElectricMotorActuatorInfoReader(ElectricMotorActuatorInfoPtr geom = ElectricMotorActuatorInfoPtr(), const AttributesList& atts=AttributesList());
    virtual ProcessElement startElement(const std::string& name, const AttributesList& atts);
    virtual bool endElement(const std::string& name);
    virtual void characters(const std::string& ch);

    inline ElectricMotorActuatorInfoPtr GetActuatorInfo() const {
        return _pinfo;
    }

protected:
    ElectricMotorActuatorInfoPtr _pinfo;
    std::stringstream _ss;
    BaseXMLReaderPtr _pcurreader;
};

typedef std::shared_ptr<ElectricMotorActuatorInfoReader> ElectricMotorActuatorInfoReaderPtr;

/// \brief reads and stores the infromation hierarchically
class OPENRAVE_API HierarchicalXMLReader : public BaseXMLReader
{
public:
    HierarchicalXMLReader(const std::string& xmlid, const AttributesList& atts);
    virtual ProcessElement startElement(const std::string& name, const AttributesList& atts);
    virtual bool endElement(const std::string& name);
    virtual void characters(const std::string& ch);
    virtual XMLReadablePtr GetReadable();
private:
    std::string _xmlid;
    std::shared_ptr<HierarchicalXMLReader> _pcurreader;
    HierarchicalXMLReadablePtr _readable;
};

typedef std::shared_ptr<HierarchicalXMLReader> HierarchicalXMLReaderPtr;

class OPENRAVE_API StreamXMLWriter : public BaseXMLWriter
{
public:
    StreamXMLWriter(const std::string& xmltag, const AttributesList& atts=AttributesList());
    const std::string& GetFormat() const;
    virtual void SetCharData(const std::string& data);
    virtual BaseXMLWriterPtr AddChild(const std::string& xmltag, const AttributesList& atts=AttributesList());
    virtual void Serialize(std::ostream& stream);

    std::list<std::shared_ptr<StreamXMLWriter> > _listchildren;
    std::string _xmltag, _data;
    AttributesList _atts;
};

typedef std::shared_ptr<StreamXMLWriter> StreamXMLWriterPtr;

} // xmlreaders
} // OpenRAVE

#endif
