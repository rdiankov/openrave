// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov (rosen.diankov@gmail.com)
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
/** \file trajectory.h
    \brief  Definition of \ref OpenRAVE::TrajectoryBase

    Automatically included with \ref openrave.h
 */
#ifndef OPENRAVE_TRAJECTORY_H
#define OPENRAVE_TRAJECTORY_H

namespace OpenRAVE {

/** \brief <b>[interface]</b> Encapsulate a time-parameterized trajectories of robot configurations. <b>If not specified, method is not multi-thread safe.</b> \arch_trajectory
    \ingroup interfaces
 */
class OPENRAVE_API TrajectoryBase : public InterfaceBase
{
public:
    TrajectoryBase(EnvironmentBasePtr penv);
    virtual ~TrajectoryBase() {
    }

    /// \brief return the static interface type this class points to (used for safe casting)
    static inline InterfaceType GetInterfaceTypeStatic() {
        return PT_Trajectory;
    }

    virtual void Init(const ConfigurationSpecification& spec) = 0;

    /** \brief Sets/inserts new waypoints in the same configuration specification as the trajectory.

        \param index The index where to start modifying the trajectory.
        \param data The data to insert, can represent multiple consecutive waypoints. data.size()/GetConfigurationSpecification().GetDOF() waypoints are added.
        \param bOverwrite If true, will overwrite the waypoints starting at index, and will insert new waypoints only if end of trajectory is reached. If false, will insert the points before index: a 0 index inserts the new data in the beginning, a GetNumWaypoints() index inserts the new data at the end.
     */
    virtual void Insert(size_t index, const std::vector<dReal>& data, bool bOverwrite=false) = 0;

    /** \brief Sets/inserts new waypoints in a \b user-given configuration specification.

        \param index The index where to start modifying the trajectory.
        \param data The data to insert, can represent multiple consecutive waypoints. data.size()/GetConfigurationSpecification().GetDOF() waypoints are added.
        \param spec the specification in which the input data come in. Depending on what data is offered, some values of this trajectory's specification might not be initialized.
        \param bOverwrite If true, will overwrite the waypoints starting at index, and will insert new waypoints only if end of trajectory is reached; if the input spec does not overwrite all the data of the trjectory spec, then the original trajectory data will not be overwritten. If false, will insert the points before index: a 0 index inserts the new data in the beginning, a GetNumWaypoints() index inserts the new data at the end.
     */
    virtual void Insert(size_t index, const std::vector<dReal>& data, const ConfigurationSpecification& spec, bool bOverwrite=false) = 0;

    /// \brief removes a number of waypoints starting at the specified index
    virtual void Remove(size_t startindex, size_t endindex) = 0;

    /** \brief samples a data point on the trajectory at a particular time

        \param data[out] the sampled point
        \param time[in] the time to sample
     */
    virtual void Sample(std::vector<dReal>& data, dReal time) const = 0;

    /** \brief samples a data point on the trajectory at a particular time and returns data for the group specified.

        The default implementation is slow, so interface developers should override it.
        \param data[out] the sampled point
        \param time[in] the time to sample
        \param spec[in] the specification format to return the data in
     */
    virtual void Sample(std::vector<dReal>& data, dReal time, const ConfigurationSpecification& spec) const;

    virtual const ConfigurationSpecification& GetConfigurationSpecification() const = 0;

    /// \brief return the number of waypoints
    virtual size_t GetNumWaypoints() const = 0;

    /** \brief return a set of waypoints in the range [startindex,endindex)

        \param startindex[in] the start index of the waypoint (included)
        \param endindex[in] the end index of the waypoint (not included)
        \param data[out] the data of the waypoint
     */
    virtual void GetWaypoints(size_t startindex, size_t endindex, std::vector<dReal>& data) const = 0;

    /** \brief return a set of waypoints in the range [startindex,endindex) in a different configuration specification.

        The default implementation is very slow, so trajectory developers should really override it.
        \param startindex[in] the start index of the waypoint (included)
        \param endindex[in] the end index of the waypoint (not included)
        \param spec[in] the specification to return the data in
        \param data[out] the data of the waypoint
     */
    virtual void GetWaypoints(size_t startindex, size_t endindex, std::vector<dReal>& data, const ConfigurationSpecification& spec) const;

    /** \brief returns one waypoint

        \param index[in] index of the waypoint. If < 0, then counting starts from the last waypoint. For example GetWaypoints(-1,data) returns the last waypoint.
        \param data[out] the data of the waypoint
     */
    inline void GetWaypoint(int index, std::vector<dReal>& data) const
    {
        int numpoints = GetNumWaypoints();
        BOOST_ASSERT(index >= -numpoints && index < numpoints);
        if( index < 0 ) {
            index += numpoints;
        }
        GetWaypoints(index,index+1,data);
    }

    /** \brief returns one waypoint

        \param index[in] index of the waypoint. If < 0, then counting starts from the last waypoint. For example GetWaypoints(-1,data) returns the last waypoint.
        \param data[out] the data of the waypoint
     */
    inline void GetWaypoint(int index, std::vector<dReal>& data, const ConfigurationSpecification& spec) const
    {
        int numpoints = GetNumWaypoints();
        BOOST_ASSERT(index >= -numpoints && index < numpoints);
        if( index < 0 ) {
            index += numpoints;
        }
        GetWaypoints(index,index+1,data,spec);
    }

    /// \brief return the duration of the trajectory in seconds
    virtual dReal GetDuration() const = 0;

    /// \brief output the trajectory in XML format
    virtual void serialize(std::ostream& O, int options=0) const;

    /// \brief initialize the trajectory
    virtual InterfaceBasePtr deserialize(std::istream& I);

    virtual void Clone(InterfaceBaseConstPtr preference, int cloningoptions);

    // Old Trajectory API

    /// \deprecated (11/10/04), please use newer API!
    class Point
    {
public:
        Point() : time(0) {
        }
        Point(const std::vector<dReal>& newq, dReal newtime) : time(newtime) {
            q = newq;
        }
        Point(const std::vector<dReal>& newq, const Transform& newtrans, dReal newtime) : time(newtime) {
            q=newq;
            trans=newtrans;
        }
        dReal time;
        std::vector<dReal> q, qdot, qtorque;
        Transform trans;
        Vector linearvel, angularvel;
    };

    /// \deprecated (11/10/04)
    typedef Point TPOINT RAVE_DEPRECATED;

    /// \deprecated (11/10/04) see \ref Sample
    virtual bool SampleTrajectory(dReal time, Point& tp) const RAVE_DEPRECATED;

    /// \deprecated (11/10/04) use \ref GetWaypoints
    virtual const std::vector<Point>& GetPoints() const RAVE_DEPRECATED;

    /// \deprecated (11/10/04) use GetConfigurationSpecification().GetDOF()
    inline int GetDOF() const RAVE_DEPRECATED {
        return GetConfigurationSpecification().GetDOF();
    }

    /// \deprecated (11/10/04) see \ref GetDuration()
    virtual dReal GetTotalDuration() const RAVE_DEPRECATED
    {
        return GetDuration();
    }

    /// \deprecated (11/10/04) see \ref serialize
    virtual bool Write(std::ostream& O, int options) const RAVE_DEPRECATED {
        serialize(O,options);
        return true;
    }

    /// \deprecated (11/10/04) see \ref deserialize
    virtual bool Read(std::istream& I, RobotBaseConstPtr) RAVE_DEPRECATED {
        deserialize(I);
        return true;
    }

    /// \deprecated (11/10/04)
    virtual int GetInterpMethod() const RAVE_DEPRECATED {
        return 0;
    }

    /// \deprecated (11/10/04) see \ref planningutils::RetimeActiveDOFTrajectory and planningutils::RetimeAffineTrajectory
    virtual bool CalcTrajTiming(RobotBasePtr probot, int interp,  bool autocalc, bool activedof, dReal fmaxvelmult=1) RAVE_DEPRECATED;

    /// \deprecated (11/10/04)
    virtual void Clear() RAVE_DEPRECATED
    {
        Remove(0,GetNumWaypoints());
    }

    /// \deprecated (11/10/04)
    virtual void AddPoint(const Point& p) RAVE_DEPRECATED;

    /// \deprecated (11/10/04)
    virtual void Reset(int dof) RAVE_DEPRECATED
    {
        Remove(0,GetNumWaypoints());
    }

    /// \deprecated (11/10/04)
    static const int TO_OneLine RAVE_DEPRECATED = 1;
    static const int TO_NoHeader RAVE_DEPRECATED = 2;
    static const int TO_IncludeTimestamps RAVE_DEPRECATED = 4;
    static const int TO_IncludeBaseTransformation RAVE_DEPRECATED = 8;
    static const int TO_IncludeVelocities RAVE_DEPRECATED = 0x10;
    static const int TO_IncludeTorques RAVE_DEPRECATED = 0x20;
    static const int TO_InterpolationMask RAVE_DEPRECATED = 0x1c0;
    static const int NONE RAVE_DEPRECATED = 0;
    static const int LINEAR RAVE_DEPRECATED = 1;
    static const int LINEAR_BLEND RAVE_DEPRECATED = 2;
    static const int CUBIC RAVE_DEPRECATED = 3;
    static const int QUINTIC RAVE_DEPRECATED = 4;
    static const int NUM_METHODS RAVE_DEPRECATED = 5;

protected:
    inline TrajectoryBasePtr shared_trajectory() {
        return boost::static_pointer_cast<TrajectoryBase>(shared_from_this());
    }
    inline TrajectoryBaseConstPtr shared_trajectory_const() const {
        return boost::static_pointer_cast<TrajectoryBase const>(shared_from_this());
    }

private:
    virtual const char* GetHash() const {
        return OPENRAVE_TRAJECTORY_HASH;
    }

    /// \deprecated (11/10/04), unfortunately necessary for back compat
    mutable std::vector<Point> __vdeprecatedpoints;
};

/// \deprecated (11/10/04)
typedef TrajectoryBase Trajectory RAVE_DEPRECATED;

} // end namespace OpenRAVE

#endif // TRAJECTORY_H
