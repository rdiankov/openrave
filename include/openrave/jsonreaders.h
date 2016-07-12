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

/** \file jsonreaders.h
    \brief classes for reading common OpenRAVE objects from JSON

    This file is optional and not automatically included with openrave.h
 */
#ifndef OPENRAVE_JSONREADERS_H
#define OPENRAVE_JSONREADERS_H

#include <openrave/openrave.h>

namespace OpenRAVE {

namespace jsonreaders {

class OPENRAVE_API StringJSONReadable : public Readable
{
public:
    StringJSONReadable(const std::string& xmlid, const std::string& data);
    virtual void Serialize(BaseJSONWriterPtr writer, int options=0) const;
    const std::string& GetData() const;
    std::string _data;
};

typedef boost::shared_ptr<StringJSONReadable> StringJSONReadablePtr;

/// \brief maintains a hierarchy of classes each containing the xml attributes and data
class OPENRAVE_API HierarchicalJSONReadable : public Readable
{
public:
    HierarchicalJSONReadable(const std::string& xmlid, const AttributesList& atts);
    virtual ~HierarchicalJSONReadable() {
    }
    virtual void Serialize(BaseJSONWriterPtr writer, int options=0) const;
    std::string _data;
    AttributesList _atts;
    std::list<boost::shared_ptr<HierarchicalJSONReadable> > _listchildren;
};

typedef boost::shared_ptr<HierarchicalJSONReadable> HierarchicalJSONReadablePtr;

/// \brief create a xml parser for trajectories
class OPENRAVE_API TrajectoryReader : public BaseJSONReader
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
    BaseJSONReaderPtr _pcurreader;
    int _datacount;
    std::vector<dReal> _vdata;
    bool _bInReadable;
};

typedef boost::shared_ptr<TrajectoryReader> TrajectoryReaderPtr;

/// \brief create a xml parser for \ref KinBody::GeometryInfo
class OPENRAVE_API GeometryInfoReader : public BaseJSONReader
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
    BaseJSONReaderPtr _pcurreader;

    bool _bOverwriteDiffuse, _bOverwriteAmbient, _bOverwriteTransparency;
    std::string _sGroupName;
};

typedef boost::shared_ptr<GeometryInfoReader> GeometryInfoReaderPtr;

/// \brief create a xml parser for \ref ElectricMotorActuatorInfo
class OPENRAVE_API ElectricMotorActuatorInfoReader : public BaseJSONReader
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
    BaseJSONReaderPtr _pcurreader;
};

typedef boost::shared_ptr<ElectricMotorActuatorInfoReader> ElectricMotorActuatorInfoReaderPtr;

/// \brief reads and stores the infromation hierarchically
class OPENRAVE_API HierarchicalJSONReader : public BaseJSONReader
{
public:
    HierarchicalJSONReader(const std::string& xmlid, const AttributesList& atts);
    virtual ProcessElement startElement(const std::string& name, const AttributesList& atts);
    virtual bool endElement(const std::string& name);
    virtual void characters(const std::string& ch);
    virtual ReadablePtr GetReadable();
private:
    std::string _xmlid;
    boost::shared_ptr<HierarchicalJSONReader> _pcurreader;
    HierarchicalJSONReadablePtr _readable;
};

typedef boost::shared_ptr<HierarchicalJSONReader> HierarchicalJSONReaderPtr;

class OPENRAVE_API StreamJSONWriter : public BaseJSONWriter
{
public:
    StreamJSONWriter(const std::string& xmltag, const AttributesList& atts=AttributesList());
    const std::string& GetFormat() const;
    virtual void SetCharData(const std::string& data);
    virtual BaseJSONWriterPtr AddChild(const std::string& xmltag, const AttributesList& atts=AttributesList());
    virtual void Serialize(std::ostream& stream);

    std::list<boost::shared_ptr<StreamJSONWriter> > _listchildren;
    std::string _xmltag, _data;
    AttributesList _atts;
};

typedef boost::shared_ptr<StreamJSONWriter> StreamJSONWriterPtr;

} // jsonreaders
} // OpenRAVE

#endif
