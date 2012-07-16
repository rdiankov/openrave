// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
};

typedef boost::shared_ptr<TrajectoryReader> TrajectoryReaderPtr;

/// \brief create a xml parser for trajectories
class OPENRAVE_API GeometryInfoReader : public BaseXMLReader
{
public:
    /// \param env the environment used to create the trajectory
    /// \param traj can optionally pass a trajectory to initialize if need to read into an existing trajectory, but the pointer can be empty
    /// \param atts attributes passed from <trajectory> tag
    GeometryInfoReader(KinBody::Link::GeometryInfoPtr geom = KinBody::Link::GeometryInfoPtr(), const AttributesList& atts=AttributesList());
    virtual ProcessElement startElement(const std::string& name, const AttributesList& atts);
    virtual bool endElement(const std::string& name);
    virtual void characters(const std::string& ch);

    inline KinBody::Link::GeometryInfoPtr GetGeometryInfo() const {
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
protected:
    KinBody::Link::GeometryInfoPtr _pgeom;
    std::stringstream _ss;
    BaseXMLReaderPtr _pcurreader;

    bool _bOverwriteDiffuse, _bOverwriteAmbient, _bOverwriteTransparency;
};

typedef boost::shared_ptr<GeometryInfoReader> GeometryInfoReaderPtr;

} // xmlreaders
} // OpenRAVE

#endif
