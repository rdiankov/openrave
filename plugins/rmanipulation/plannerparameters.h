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

class ExplorationParameters : public PlannerBase::PlannerParameters
{
public:
    ExplorationParameters() : _fExploreProb(0), _nExpectedDataSize(100), _bProcessingExploration(false) {
        _vXMLParameters.push_back("exploreprob");
        _vXMLParameters.push_back("expectedsize");
    }

    dReal _fExploreProb;
    int _nExpectedDataSize;

protected:
    bool _bProcessingExploration;
    // save the extra data to XML
    virtual bool serialize(std::ostream& O) const
    {
        if( !PlannerParameters::serialize(O) ) {
            return false;
        }
        O << "<exploreprob>" << _fExploreProb << "</exploreprob>" << endl;
        O << "<expectedsize>" << _nExpectedDataSize << "</expectedsize>" << endl;
        return !!O;
    }

    ProcessElement startElement(const std::string& name, const AttributesList& atts)
    {
        if( _bProcessingExploration ) {
            return PE_Ignore;
        }
        switch( PlannerBase::PlannerParameters::startElement(name,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }

        _bProcessingExploration = name=="exploreprob"||name=="expectedsize";
        return _bProcessingExploration ? PE_Support : PE_Pass;
    }

    // called at the end of every XML tag, _ss contains the data
    virtual bool endElement(const std::string& name)
    {
        // _ss is an internal stringstream that holds the data of the tag
        if( _bProcessingExploration ) {
            if( name == "exploreprob") {
                _ss >> _fExploreProb;
            }
            else if( name == "expectedsize" ) {
                _ss >> _nExpectedDataSize;
            }
            else {
                RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
            }
            _bProcessingExploration = false;
            return false;
        }

        // give a chance for the default parameters to get processed
        return PlannerParameters::endElement(name);
    }
};

class RAStarParameters : public PlannerBase::PlannerParameters
{
public:
    RAStarParameters() : fRadius(0.1f), fDistThresh(0.03f), fGoalCoeff(1), nMaxChildren(5), nMaxSampleTries(10), _bProcessingRA(false) {
        _vXMLParameters.push_back("radius");
        _vXMLParameters.push_back("distthresh");
        _vXMLParameters.push_back("goalcoeff");
        _vXMLParameters.push_back("maxchildren");
        _vXMLParameters.push_back("maxsampletries");
    }

    dReal fRadius;          ///< _pDistMetric thresh is the radius that children must be within parents
    dReal fDistThresh;      ///< gamma * _pDistMetric->thresh is the sampling radius
    dReal fGoalCoeff;       ///< balancees exploratino vs cost
    int nMaxChildren;       ///< limit on number of children
    int nMaxSampleTries;     ///< max sample tries before giving up on creating a child
protected:
    bool _bProcessingRA;
    virtual bool serialize(std::ostream& O) const
    {
        if( !PlannerParameters::serialize(O) ) {
            return false;
        }
        O << "<radius>" << fRadius << "</radius>" << endl;
        O << "<distthresh>" << fDistThresh << "</distthresh>" << endl;
        O << "<goalcoeff>" << fGoalCoeff << "</goalcoeff>" << endl;
        O << "<maxchildren>" << nMaxChildren << "</maxchildren>" << endl;
        O << "<maxsampletries>" << nMaxSampleTries << "</maxsampletries>" << endl;

        return !!O;
    }

    ProcessElement startElement(const std::string& name, const AttributesList& atts)
    {
        if( _bProcessingRA ) {
            return PE_Ignore;
        }
        switch( PlannerBase::PlannerParameters::startElement(name,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }
        _bProcessingRA = name=="radius"||name=="distthresh"||name=="goalcoeff"||name=="maxchildren"||name=="maxsampletries";
        return _bProcessingRA ? PE_Support : PE_Pass;
    }
    virtual bool endElement(const string& name)
    {
        if( _bProcessingRA ) {
            if( name == "radius") {
                _ss >> fRadius;
            }
            else if( name == "distthresh") {
                _ss >> fDistThresh;
            }
            else if( name == "goalcoeff") {
                _ss >> fGoalCoeff;
            }
            else if( name == "maxchildren") {
                _ss >> nMaxChildren;
            }
            else if( name == "maxsampletries") {
                _ss >> nMaxSampleTries;
            }
            else {
                RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
            }
            _bProcessingRA = false;
            return false;
        }
        // give a chance for the default parameters to get processed
        return PlannerParameters::endElement(name);
    }
};

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
        if( !PlannerParameters::serialize(O) ) {
            return false;
        }
        O << "<grasps>" << _vgrasps.size() << " ";
        FOREACHC(it, _vgrasps) {
            O << *it << " ";
        }
        O << "</grasps>" << endl;
        O << "<target>" << (!!_ptarget ? _ptarget->GetEnvironmentId() : 0) << "</target>" << endl;
        O << "<numgradsamples>" << _nGradientSamples << "</numgradsamples>" << endl;
        O << "<visgraspthresh>" << _fVisibiltyGraspThresh << "</visgraspthresh>" << endl;
        O << "<graspdistthresh>" << _fGraspDistThresh << "</graspdistthresh>" << endl;
        return !!O;
    }

    ProcessElement startElement(const std::string& name, const AttributesList& atts)
    {
        if( _bProcessingGS ) {
            return PE_Ignore;
        }
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
            else if( name == "numgradsamples" ) {
                _ss >> _nGradientSamples;
            }
            else if( name == "visgraspthresh" ) {
                _ss >> _fVisibiltyGraspThresh;
            }
            else if( name == "graspdistthresh") {
                _ss >> _fGraspDistThresh;
            }
            else {
                RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
            }
            _bProcessingGS = false;
            return false;
        }

        // give a chance for the default parameters to get processed
        return PlannerParameters::endElement(name);
    }
};

class GraspParameters : public PlannerBase::PlannerParameters
{
public:
    GraspParameters(EnvironmentBasePtr penv) : PlannerBase::PlannerParameters(), fstandoff(0), ftargetroll(0), vtargetdirection(0,0,1), btransformrobot(false), breturntrajectory(false), bonlycontacttarget(true), btightgrasp(false), bavoidcontact(false), fcoarsestep(0.1f), ffinestep(0.001f), ftranslationstepmult(0.1f), fgraspingnoise(0), _penv(penv) {
        _vXMLParameters.push_back("fstandoff");
        _vXMLParameters.push_back("targetbody");
        _vXMLParameters.push_back("ftargetroll");
        _vXMLParameters.push_back("vtargetdirection");
        _vXMLParameters.push_back("vtargetposition");
        _vXMLParameters.push_back("vmanipulatordirection");
        _vXMLParameters.push_back("btransformrobot");
        _vXMLParameters.push_back("breturntrajectory");
        _vXMLParameters.push_back("bonlycontacttarget");
        _vXMLParameters.push_back("btightgrasp");
        _vXMLParameters.push_back("bavoidcontact");
        _vXMLParameters.push_back("vavoidlinkgeometry");
        _vXMLParameters.push_back("fcoarsestep");
        _vXMLParameters.push_back("ffinestep");
        _vXMLParameters.push_back("ftranslationstepmult");
        _vXMLParameters.push_back("fgraspingnoise");
        _bProcessingGrasp = false;
    }

    dReal fstandoff;     ///< start closing fingers when at this distance
    KinBodyPtr targetbody;     ///< the target that will be grasped, all parameters will be in this coordinate system. if not present, then below transformations are in absolute coordinate system.
    dReal ftargetroll;     ///< rotate the hand about the palm normal (if one exists) by this many radians
    Vector vtargetdirection;     ///< direction in target space to approach object from
    Vector vtargetposition;     ///< position in target space to start approaching (if in collision with target, gets backed up)
    Vector vmanipulatordirection; ///< a direction for the gripper to face at when approaching (in the manipulator coordinate system)
    bool btransformrobot;     ///< if true sets the base link of the robot given the above transformation parameters. If there is an active manipulator
    bool breturntrajectory;     ///< if true, returns how the individual fingers moved instead of just the final grasp
    bool bonlycontacttarget;     ///< if true, then grasp is successful only if contact is made with the target
    bool btightgrasp;     ///< This is tricky, but basically if true will also move the basic link along the negative axes of some of the joints to get a tighter fit.
    bool bavoidcontact;     ///< if true, will return a final robot configuration right before contact is made.
    vector<string> vavoidlinkgeometry;     ///< list of links on the robot to avoid collisions with (for exmaple, sensors)

    dReal fcoarsestep;      ///< step for coarse planning (in radians)
    dReal ffinestep;     ///< step for fine planning (in radians), THIS STEP MUST BE VERY SMALL OR THE COLLISION CHECKER GIVES WILDLY BOGUS RESULTS
    dReal ftranslationstepmult;     ///< multiplication factor for translational movements of the hand or joints

    dReal fgraspingnoise;     ///< random undeterministic noise to add to the target object, represents the max possible displacement of any point on the object (noise added after global direction and start have been determined)
protected:
    EnvironmentBasePtr _penv;     ///< environment target belongs to
    bool _bProcessingGrasp;
    // save the extra data to XML
    virtual bool serialize(std::ostream& O) const
    {
        if( !PlannerParameters::serialize(O) ) {
            return false;
        }
        O << "<fstandoff>" << fstandoff << "</fstandoff>" << endl;
        O << "<targetbody>" << (int)(!targetbody ? 0 : targetbody->GetEnvironmentId()) << "</targetbody>" << endl;
        O << "<ftargetroll>" << ftargetroll << "</ftargetroll>" << endl;
        O << "<vtargetdirection>" << vtargetdirection << "</vtargetdirection>" << endl;
        O << "<vtargetposition>" << vtargetposition << "</vtargetposition>" << endl;
        O << "<vmanipulatordirection>" << vmanipulatordirection << "</vmanipulatordirection>" << endl;
        O << "<btransformrobot>" << btransformrobot << "</btransformrobot>" << endl;
        O << "<breturntrajectory>" << breturntrajectory << "</breturntrajectory>" << endl;
        O << "<bonlycontacttarget>" << bonlycontacttarget << "</bonlycontacttarget>" << endl;
        O << "<btightgrasp>" << btightgrasp << "</btightgrasp>" << endl;
        O << "<bavoidcontact>" << bavoidcontact << "</bavoidcontact>" << endl;
        O << "<vavoidlinkgeometry>" << endl;
        FOREACHC(it,vavoidlinkgeometry) {
            O << *it << " ";
        }
        O << "</vavoidlinkgeometry>" << endl;
        O << "<fcoarsestep>" << fcoarsestep << "</fcoarsestep>" << endl;
        O << "<ffinestep>" << ffinestep << "</ffinestep>" << endl;
        O << "<ftranslationstepmult>" << ftranslationstepmult << "</ftranslationstepmult>" << endl;
        O << "<fgraspingnoise>" << fgraspingnoise << "</fgraspingnoise>" << endl;
        return !!O;
    }

    ProcessElement startElement(const std::string& name, const AttributesList& atts)
    {
        if( _bProcessingGrasp ) {
            return PE_Ignore;
        }
        switch( PlannerBase::PlannerParameters::startElement(name,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }
        if( name == "vavoidlinkgeometry" ) {
            vavoidlinkgeometry.resize(0);
            return PE_Support;
        }

        boost::array<string,16> tags = {{"fstandoff","targetbody","ftargetroll","vtargetdirection","vtargetposition","vmanipulatordirection", "btransformrobot","breturntrajectory","bonlycontacttarget","btightgrasp","bavoidcontact","vavoidlinkgeometry","fcoarsestep","ffinestep","ftranslationstepmult","fgraspingnoise"}};
        return find(tags.begin(),tags.end(),name) == tags.end() ? PE_Pass : PE_Support;
    }

    // called at the end of every XML tag, _ss contains the data
    virtual bool endElement(const std::string& name)
    {
        // _ss is an internal stringstream that holds the data of the tag
        if( _bProcessingGrasp ) {
            if( name == "vavoidlinkgeometry" ) {
                vavoidlinkgeometry = vector<string>((istream_iterator<string>(_ss)), istream_iterator<string>());
            }
            else if( name == "fstandoff") {
                _ss >> fstandoff;
            }
            else if( name == "targetbody") {
                int id = 0;
                _ss >> id;
                targetbody = _penv->GetBodyFromEnvironmentId(id);
            }
            else if( name == "ftargetroll") {
                _ss >> ftargetroll;
            }
            else if( name == "vtargetdirection") {
                _ss >> vtargetdirection;
                vtargetdirection.normalize3();
            }
            else if( name == "vtargetposition") {
                _ss >> vtargetposition;
            }
            else if( name == "vmanipulatordirection") {
                _ss >> vmanipulatordirection;
            }
            else if( name == "btransformrobot") {
                _ss >> btransformrobot;
            }
            else if( name == "breturntrajectory") {
                _ss >> breturntrajectory;
            }
            else if( name == "bonlycontacttarget") {
                _ss >> bonlycontacttarget;
            }
            else if( name == "btightgrasp" ) {
                _ss >> btightgrasp;
            }
            else if( name == "bavoidcontact" ) {
                _ss >> bavoidcontact;
            }
            else if( name == "fcoarsestep" ) {
                _ss >> fcoarsestep;
            }
            else if( name == "ffinestep" ) {
                _ss >> ffinestep;
            }
            else if( name == "fgraspingnoise" ) {
                _ss >> fgraspingnoise;
            }
            else if( name == "ftranslationstepmult" ) {
                _ss >> ftranslationstepmult;
            }
            else {
                RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
            }
            _bProcessingGrasp = false;
            return false;
        }

        // give a chance for the default parameters to get processed
        return PlannerParameters::endElement(name);
    }
};

class TrajectoryTimingParameters : public PlannerBase::PlannerParameters
{
public:
    TrajectoryTimingParameters() : _interpolation("linear"), _hastimestamps(false), _pointtolerance(0.001), _bProcessing(false) {
        _vXMLParameters.push_back("interpolation");
        _vXMLParameters.push_back("hastimestamps");
        _vXMLParameters.push_back("pointtolerance");
    }

    string _interpolation;
    bool _hastimestamps;
    dReal _pointtolerance;

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

        _bProcessing = name=="interpolation" || name=="hastimestamps" || name=="pointtolerance";
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
