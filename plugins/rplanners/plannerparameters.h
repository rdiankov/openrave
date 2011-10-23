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
/** \file planner.h
    \brief Planning related defintions.
 */
#ifndef OPENRAVE_PLANNER_PARAMETERS_H
#define OPENRAVE_PLANNER_PARAMETERS_H

class GraspSetParameters : public PlannerBase::PlannerParameters
{
public:
    GraspSetParameters(EnvironmentBasePtr penv) : _nGradientSamples(5), _fVisibiltyGraspThresh(0), _fGraspDistThresh(1.4f), _penv(penv),_bProcessingGS(false) {
        _vXMLParameters.push_back("grasps");
        _vXMLParameters.push_back("target");
        _vXMLParameters.push_back("numgradsamples");
        _vXMLParameters.push_back("visgraspthresh");
        _vXMLParameters.push_back("graspdistthresh");
    }

    vector<Transform> _vgrasps;     ///< grasps with respect to the target object
    KinBodyPtr _ptarget;
    int _nGradientSamples;
    dReal _fVisibiltyGraspThresh;     ///< if current grasp is less than this threshold, then visibilty is not checked
    dReal _fGraspDistThresh;     ///< target grasps beyond this distance are ignored

protected:
    EnvironmentBasePtr _penv;
    bool _bProcessingGS;
    virtual bool serialize(std::ostream& O) const
    {
        if( !PlannerParameters::serialize(O) )
            return false;
        O << "<grasps>" << _vgrasps.size() << " ";
        FOREACHC(it, _vgrasps)
        O << *it << " ";
        O << "</grasps>" << endl;
        O << "<target>" << (!!_ptarget ? _ptarget->GetEnvironmentId() : 0) << "</target>" << endl;
        O << "<numgradsamples>" << _nGradientSamples << "</numgradsamples>" << endl;
        O << "<visgraspthresh>" << _fVisibiltyGraspThresh << "</visgraspthresh>" << endl;
        O << "<graspdistthresh>" << _fGraspDistThresh << "</graspdistthresh>" << endl;
        return !!O;
    }

    ProcessElement startElement(const std::string& name, const AttributesList& atts)
    {
        if( _bProcessingGS )
            return PE_Ignore;
        switch( PlannerBase::PlannerParameters::startElement(name,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }

        _bProcessingGS = name=="grasps"||name=="target"||name=="numgradsamples"||name=="visgraspthresh"||name=="graspdistthresh";
        return _bProcessingGS ? PE_Support : PE_Pass;
    }

    virtual bool endElement(const string& name)
    {
        if( _bProcessingGS ) {
            if( name == "grasps" ) {
                int ngrasps=0;
                _ss >> ngrasps;
                _vgrasps.resize(ngrasps);
                FOREACH(it, _vgrasps) {
                    _ss >> *it;
                }
            }
            else if( name == "target" ) {
                int id = 0;
                _ss >> id;
                _ptarget = _penv->GetBodyFromEnvironmentId(id);
            }
            else if( name == "numgradsamples" )
                _ss >> _nGradientSamples;
            else if( name == "visgraspthresh" )
                _ss >> _fVisibiltyGraspThresh;
            else if( name == "graspdistthresh")
                _ss >> _fGraspDistThresh;
            else
                RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
            _bProcessingGS = false;
            return false;
        }

        // give a chance for the default parameters to get processed
        return PlannerParameters::endElement(name);
    }
};

class BasicRRTParameters : public PlannerBase::PlannerParameters
{
public:
    BasicRRTParameters() : _fGoalBiasProb(0.05f), _bProcessing(false) {
        _vXMLParameters.push_back("goalbias");
    }

    dReal _fGoalBiasProb;

protected:
    bool _bProcessing;
    virtual bool serialize(std::ostream& O) const
    {
        if( !PlannerParameters::serialize(O) ) {
            return false;
        }
        O << "<goalbias>" << _fGoalBiasProb << "</goalbias>" << endl;
        return !!O;
    }

    ProcessElement startElement(const std::string& name, const AttributesList& atts)
    {
        if( _bProcessing ) {
            return PE_Ignore;
        }
        switch( PlannerBase::PlannerParameters::startElement(name,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }

        _bProcessing = name=="goalbias";
        return _bProcessing ? PE_Support : PE_Pass;
    }

    virtual bool endElement(const string& name)
    {
        if( _bProcessing ) {
            if( name == "goalbias") {
                _ss >> _fGoalBiasProb;
            }
            else {
                RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
            }
            _bProcessing = false;
            return false;
        }

        // give a chance for the default parameters to get processed
        return PlannerParameters::endElement(name);
    }
};

class TrajectoryTimingParameters : public PlannerBase::PlannerParameters
{
public:
    TrajectoryTimingParameters() : _interpolation("linear"), _pointtolerance(0.001), _hastimestamps(false), _outputaccelchanges(true), _bProcessing(false) {
        _vXMLParameters.push_back("interpolation");
        _vXMLParameters.push_back("hastimestamps");
        _vXMLParameters.push_back("pointtolerance");
        _vXMLParameters.push_back("outputaccelchanges");
    }

    string _interpolation;
    dReal _pointtolerance;
    bool _hastimestamps;
    bool _outputaccelchanges;
protected:
    bool _bProcessing;
    virtual bool serialize(std::ostream& O) const
    {
        if( !PlannerParameters::serialize(O) ) {
            return false;
        }
        O << "<interpolation>" << _interpolation << "</interpolation>" << endl;
        O << "<hastimestamps>" << _hastimestamps << "</hastimestamps>" << endl;
        O << "<pointtolerance>" << _pointtolerance << "</pointtolerance>" << endl;
        O << "<outputaccelchanges>" << _outputaccelchanges << "</outputaccelchanges>" << endl;
        return !!O;
    }

    ProcessElement startElement(const std::string& name, const AttributesList& atts)
    {
        if( _bProcessing ) {
            return PE_Ignore;
        }
        switch( PlannerBase::PlannerParameters::startElement(name,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }

        _bProcessing = name=="interpolation" || name=="hastimestamps" || name=="pointtolerance" || name=="outputaccelchanges";
        return _bProcessing ? PE_Support : PE_Pass;
    }

    virtual bool endElement(const string& name)
    {
        if( _bProcessing ) {
            if( name == "interpolation") {
                _ss >> _interpolation;
            }
            else if( name == "hastimestamps" ) {
                _ss >> _hastimestamps;
            }
            else if( name == "pointtolerance" ) {
                _ss >> _pointtolerance;
            }
            else if( name == "outputaccelchanges" ) {
                _ss >> _outputaccelchanges;
            }
            else {
                RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
            }
            _bProcessing = false;
            return false;
        }

        // give a chance for the default parameters to get processed
        return PlannerParameters::endElement(name);
    }
};

typedef boost::shared_ptr<TrajectoryTimingParameters> TrajectoryTimingParametersPtr;
typedef boost::shared_ptr<TrajectoryTimingParameters const> TrajectoryTimingParametersConstPtr;

class WorkspaceTrajectoryParameters : public PlannerBase::PlannerParameters
{
public:
    WorkspaceTrajectoryParameters(EnvironmentBasePtr penv) : maxdeviationangle(0.15*PI), maintaintiming(false), greedysearch(true), ignorefirstcollision(0), minimumcompletetime(1e30f), _penv(penv), _bProcessing(false) {
        _vXMLParameters.push_back("maxdeviationangle");
        _vXMLParameters.push_back("maintaintiming");
        _vXMLParameters.push_back("greedysearch");
        _vXMLParameters.push_back("ignorefirstcollision");
        _vXMLParameters.push_back("minimumcompletetime");
        _vXMLParameters.push_back("workspacetraj");
        _vXMLParameters.push_back("conveyorspeed");
    }

    dReal maxdeviationangle;     ///< the maximum angle the next iksolution can deviate from the expected direction computed by the jacobian
    bool maintaintiming;     ///< maintain timing with input trajectory
    bool greedysearch;     ///< if true, will greeidly choose solutions (can possibly fail even a solution exists)
    dReal ignorefirstcollision;     ///< if > 0, will allow the robot to be in environment collision for the initial 'ignorefirstcollision' seconds of the trajectory. Once the robot gets out of collision, it will execute its normal following phase until it gets into collision again. This option is used when lifting objects from a surface, where the object is already in collision with the surface.
    dReal minimumcompletetime;     ///< specifies the minimum trajectory that must be followed for planner to declare success. If 0, then the entire trajectory has to be followed.
    TrajectoryBasePtr workspacetraj;     ///< workspace trajectory
    Vector conveyorspeed; ///< velocity of the coordinate system. used if object is on is moving at a constant speed on a conveyor

protected:
    EnvironmentBasePtr _penv;
    bool _bProcessing;
    // save the extra data to XML
    virtual bool serialize(std::ostream& O) const
    {
        if( !PlannerParameters::serialize(O) ) {
            return false;
        }
        O << "<maxdeviationangle>" << maxdeviationangle << "</maxdeviationangle>" << endl;
        O << "<maintaintiming>" << maintaintiming << "</maintaintiming>" << endl;
        O << "<greedysearch>" << greedysearch << "</greedysearch>" << endl;
        O << "<ignorefirstcollision>" << ignorefirstcollision << "</ignorefirstcollision>" << endl;
        O << "<minimumcompletetime>" << minimumcompletetime << "</minimumcompletetime>" << endl;
        if( !!workspacetraj ) {
            O << "<workspacetraj><![CDATA[";
            workspacetraj->serialize(O);
            O << "]]></workspacetraj>" << endl;
        }
        O << "<conveyorspeed>" << conveyorspeed << "</conveyorspeed>" << endl;
        return !!O;
    }

    ProcessElement startElement(const std::string& name, const AttributesList& atts)
    {
        if( _bProcessing ) {
            return PE_Ignore;
        }
        switch( PlannerBase::PlannerParameters::startElement(name,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }
        _bProcessing = name=="maxdeviationangle" || name=="maintaintiming" || name=="greedysearch" || name=="ignorefirstcollision" || name=="minimumcompletetime" || name=="workspacetraj" || name == "conveyorspeed";
        return _bProcessing ? PE_Support : PE_Pass;
    }

    // called at the end of every XML tag, _ss contains the data
    virtual bool endElement(const std::string& name)
    {
        // _ss is an internal stringstream that holds the data of the tag
        if( _bProcessing ) {
            if( name == "maxdeviationangle") {
                _ss >> maxdeviationangle;
            }
            else if( name == "maintaintiming" ) {
                _ss >> maintaintiming;
            }
            else if( name == "greedysearch" ) {
                _ss >> greedysearch;
            }
            else if( name == "ignorefirstcollision" ) {
                _ss >> ignorefirstcollision;
            }
            else if( name == "minimumcompletetime" ) {
                _ss >> minimumcompletetime;
            }
            else if( name == "conveyorspeed" ) {
                _ss >> conveyorspeed;
            }
            else if( name == "workspacetraj" ) {
                if( !workspacetraj ) {
                    workspacetraj = RaveCreateTrajectory(_penv,"");
                }
                workspacetraj->deserialize(_ss);
            }
            else {
                RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
            }
            _bProcessing = false;
            return false;
        }
        // give a chance for the default parameters to get processed
        return PlannerParameters::endElement(name);
    }
};

typedef boost::shared_ptr<WorkspaceTrajectoryParameters> WorkspaceTrajectoryParametersPtr;
typedef boost::shared_ptr<WorkspaceTrajectoryParameters const> WorkspaceTrajectoryParametersConstPtr;

#endif
