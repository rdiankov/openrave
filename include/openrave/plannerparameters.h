// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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
/** \file plannerparameters
    \brief Specific planner related parameters information
 */
#ifndef OPENRAVE_PLANNER_PARAMETERS_H
#define OPENRAVE_PLANNER_PARAMETERS_H

#include <openrave/planningutils.h>

namespace OpenRAVE {

class OPENRAVE_API ExplorationParameters : public PlannerBase::PlannerParameters
{
public:
    ExplorationParameters() : _fExploreProb(0), _nExpectedDataSize(100), _bProcessingExploration(false) {
        _vXMLParameters.push_back("exploreprob");
        _vXMLParameters.push_back("expectedsize");
    }

    dReal _fExploreProb; ///< explore close to the neighbors already added
    int _nExpectedDataSize; ///< the expected number of nodes of the RRT tree to expand to before returning a solution. This allows the RRT tree to build up

protected:
    bool _bProcessingExploration;
    // save the extra data to XML
    virtual bool serialize(std::ostream& O, int options=0) const
    {
        if( !PlannerParameters::serialize(O,options&~1) ) { // skip writing extra
            return false;
        }
        O << "<exploreprob>" << _fExploreProb << "</exploreprob>" << std::endl;
        O << "<expectedsize>" << _nExpectedDataSize << "</expectedsize>" << std::endl;
        if( !(options & 1) ) {
            O << _sExtraParameters << std::endl;
        }
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

typedef boost::shared_ptr<ExplorationParameters> ExplorationParametersPtr;
typedef boost::shared_ptr<ExplorationParameters const> ExplorationParametersConstPtr;

class OPENRAVE_API RAStarParameters : public PlannerBase::PlannerParameters
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
    virtual bool serialize(std::ostream& O, int options) const
    {
        if( !PlannerParameters::serialize(O,options&~1) ) {
            return false;
        }
        O << "<radius>" << fRadius << "</radius>" << std::endl;
        O << "<distthresh>" << fDistThresh << "</distthresh>" << std::endl;
        O << "<goalcoeff>" << fGoalCoeff << "</goalcoeff>" << std::endl;
        O << "<maxchildren>" << nMaxChildren << "</maxchildren>" << std::endl;
        O << "<maxsampletries>" << nMaxSampleTries << "</maxsampletries>" << std::endl;
        if( !(options & 1) ) {
            O << _sExtraParameters << std::endl;
        }

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
    virtual bool endElement(const std::string& name)
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

class OPENRAVE_API GraspSetParameters : public PlannerBase::PlannerParameters
{
public:
    GraspSetParameters(EnvironmentBasePtr penv) : _nGradientSamples(5), _fVisibiltyGraspThresh(0), _fGraspDistThresh(1.4f), _penv(penv),_bProcessingGS(false) {
        _vXMLParameters.push_back("grasps");
        _vXMLParameters.push_back("target");
        _vXMLParameters.push_back("numgradsamples");
        _vXMLParameters.push_back("visgraspthresh");
        _vXMLParameters.push_back("graspdistthresh");
    }

    std::vector<Transform> _vgrasps;     ///< grasps with respect to the target object
    KinBodyPtr _ptarget;
    int _nGradientSamples;
    dReal _fVisibiltyGraspThresh;     ///< if current grasp is less than this threshold, then visibilty is not checked
    dReal _fGraspDistThresh;     ///< target grasps beyond this distance are ignored

protected:
    EnvironmentBasePtr _penv;
    bool _bProcessingGS;
    virtual bool serialize(std::ostream& O, int options=0) const
    {
        if( !PlannerParameters::serialize(O,options&~1) ) {
            return false;
        }
        O << "<grasps>" << _vgrasps.size() << " ";
        for(std::vector<Transform>::const_iterator it = _vgrasps.begin(); it != _vgrasps.end(); ++it) {
            O << *it << " ";
        }
        O << "</grasps>" << std::endl;
        O << "<target>" << (!!_ptarget ? _ptarget->GetEnvironmentId() : 0) << "</target>" << std::endl;
        O << "<numgradsamples>" << _nGradientSamples << "</numgradsamples>" << std::endl;
        O << "<visgraspthresh>" << _fVisibiltyGraspThresh << "</visgraspthresh>" << std::endl;
        O << "<graspdistthresh>" << _fGraspDistThresh << "</graspdistthresh>" << std::endl;
        if( !(options & 1) ) {
            O << _sExtraParameters << std::endl;
        }
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

    virtual bool endElement(const std::string& name)
    {
        if( _bProcessingGS ) {
            if( name == "grasps" ) {
                int ngrasps=0;
                _ss >> ngrasps;
                _vgrasps.resize(ngrasps);
                for(std::vector<Transform>::iterator it = _vgrasps.begin(); it != _vgrasps.end(); ++it) {
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

class OPENRAVE_API GraspParameters : public PlannerBase::PlannerParameters
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
        _vXMLParameters.push_back("vintersectplane");
        _vXMLParameters.push_back("vordereddofindices");
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
    std::vector<std::string> vavoidlinkgeometry;     ///< list of links on the robot to avoid collisions with (for exmaple, sensors)

    dReal fcoarsestep;      ///< step for coarse planning (in radians)
    dReal ffinestep;     ///< step for fine planning (in radians), THIS STEP MUST BE VERY SMALL OR THE COLLISION CHECKER GIVES WILDLY BOGUS RESULTS
    dReal ftranslationstepmult;     ///< multiplication factor for translational movements of the hand or joints
    dReal fgraspingnoise;     ///< random undeterministic noise to add to the target object, represents the max possible displacement of any point on the object (noise added after global direction and start have been determined)
    Vector vintersectplane; ///< if norm > 0, then the manipulator transform has to be on the plane for grabbing to work. This is mutually exclusive from the standoff.

    std::vector<int> vordereddofindices; ///< if specified, will move fingers in this order instead of the order specified by robot->GetActiveDOFIndices().

protected:
    EnvironmentBasePtr _penv;     ///< environment target belongs to
    bool _bProcessingGrasp;
    // save the extra data to XML
    virtual bool serialize(std::ostream& O, int options=0) const
    {
        if( !PlannerParameters::serialize(O, options&~1) ) {
            return false;
        }
        O << "<fstandoff>" << fstandoff << "</fstandoff>" << std::endl;
        O << "<targetbody>" << (int)(!targetbody ? 0 : targetbody->GetEnvironmentId()) << "</targetbody>" << std::endl;
        O << "<ftargetroll>" << ftargetroll << "</ftargetroll>" << std::endl;
        O << "<vtargetdirection>" << vtargetdirection << "</vtargetdirection>" << std::endl;
        O << "<vtargetposition>" << vtargetposition << "</vtargetposition>" << std::endl;
        O << "<vmanipulatordirection>" << vmanipulatordirection << "</vmanipulatordirection>" << std::endl;
        O << "<btransformrobot>" << btransformrobot << "</btransformrobot>" << std::endl;
        O << "<breturntrajectory>" << breturntrajectory << "</breturntrajectory>" << std::endl;
        O << "<bonlycontacttarget>" << bonlycontacttarget << "</bonlycontacttarget>" << std::endl;
        O << "<btightgrasp>" << btightgrasp << "</btightgrasp>" << std::endl;
        O << "<bavoidcontact>" << bavoidcontact << "</bavoidcontact>" << std::endl;
        O << "<vavoidlinkgeometry>" << std::endl;
        for(std::vector<std::string>::const_iterator it = vavoidlinkgeometry.begin(); it != vavoidlinkgeometry.end(); ++it) {
            O << *it << " ";
        }
        O << "</vavoidlinkgeometry>" << std::endl;
        O << "<fcoarsestep>" << fcoarsestep << "</fcoarsestep>" << std::endl;
        O << "<ffinestep>" << ffinestep << "</ffinestep>" << std::endl;
        O << "<ftranslationstepmult>" << ftranslationstepmult << "</ftranslationstepmult>" << std::endl;
        O << "<fgraspingnoise>" << fgraspingnoise << "</fgraspingnoise>" << std::endl;
        O << "<vintersectplane>" << vintersectplane << "</vintersectplane>" << std::endl;
        O << "<vordereddofindices>" << std::endl;
        for(std::vector<int>::const_iterator it = vordereddofindices.begin(); it != vordereddofindices.end(); ++it) {
            O << *it << " ";
        }
        O << "</vordereddofindices>" << std::endl;
        if( !(options & 1) ) {
            O << _sExtraParameters << std::endl;
        }
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

        static boost::array<std::string,18> tags = {{"fstandoff","targetbody","ftargetroll","vtargetdirection","vtargetposition","vmanipulatordirection", "btransformrobot","breturntrajectory","bonlycontacttarget","btightgrasp","bavoidcontact","vavoidlinkgeometry","fcoarsestep","ffinestep","ftranslationstepmult","fgraspingnoise","vintersectplane","vordereddofindices"}};
        _bProcessingGrasp = find(tags.begin(),tags.end(),name) != tags.end();
        return _bProcessingGrasp ? PE_Support : PE_Pass;
    }

    // called at the end of every XML tag, _ss contains the data
    virtual bool endElement(const std::string& name)
    {
        // _ss is an internal stringstream that holds the data of the tag
        if( _bProcessingGrasp ) {
            if( name == "vavoidlinkgeometry" ) {
                vavoidlinkgeometry = std::vector<std::string>((std::istream_iterator<std::string>(_ss)), std::istream_iterator<std::string>());
            }
            else if( name == "vordereddofindices" ) {
                vordereddofindices = std::vector<int>((std::istream_iterator<int>(_ss)), std::istream_iterator<int>());
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
            else if( name == "vintersectplane" ) {
                _ss >> vintersectplane;
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

typedef boost::shared_ptr<GraspParameters> GraspParametersPtr;
typedef boost::shared_ptr<GraspParameters const> GraspParametersConstPtr;

/** \brief parameters for timing/smoothing trajectories

    PlannerBase::PlannerParameters::_fStepLength is used for the control time of the robot identifying the discretization of the trajectory time. if 0, will ignore discretization of time.
 **/
class OPENRAVE_API TrajectoryTimingParameters : public PlannerBase::PlannerParameters
{
public:
    TrajectoryTimingParameters() : _interpolation(""), _pointtolerance(0.2), _hastimestamps(false), _hasvelocities(false), _outputaccelchanges(true), _multidofinterp(0), verifyinitialpath(1), _bProcessing(false) {
        _fStepLength = 0; // reset to 0 since it is being used
        _vXMLParameters.push_back("interpolation");
        _vXMLParameters.push_back("hastimestamps");
        _vXMLParameters.push_back("hasvelocities");
        _vXMLParameters.push_back("pointtolerance");
        _vXMLParameters.push_back("outputaccelchanges");
        _vXMLParameters.push_back("multidofinterp");
        _vXMLParameters.push_back("verifyinitialpath");
    }

    std::string _interpolation;
    dReal _pointtolerance; ///< multiple of dof resolutions to set on discretization tolerance
    bool _hastimestamps, _hasvelocities;
    bool _outputaccelchanges; ///< if true, will output a waypoint every time a DOF changes its acceleration, this allows a trajectory be executed without knowing the max velocities/accelerations. If false, will just output the waypoints.
    int _multidofinterp; ///< if 1, will always force the max acceleration of the robot when retiming rather than using lesser acceleration whenever possible. if 0, will compute minimum acceleration. If 2, will match acceleration ramps of all dofs.
    int verifyinitialpath; ///< if 0 then does not verify whether the path given as input is in collision

protected:
    bool _bProcessing;
    virtual bool serialize(std::ostream& O, int options=0) const
    {
        if( !PlannerParameters::serialize(O, options&~1) ) {
            return false;
        }
        O << "<interpolation>" << _interpolation << "</interpolation>" << std::endl;
        O << "<hastimestamps>" << _hastimestamps << "</hastimestamps>" << std::endl;
        O << "<hasvelocities>" << _hasvelocities << "</hasvelocities>" << std::endl;
        O << "<pointtolerance>" << _pointtolerance << "</pointtolerance>" << std::endl;
        O << "<outputaccelchanges>" << _outputaccelchanges << "</outputaccelchanges>" << std::endl;
        O << "<multidofinterp>" << _multidofinterp << "</multidofinterp>" << std::endl;
        O << "<verifyinitialpath>" << verifyinitialpath  << "</verifyinitialpath>" << std::endl;
        if( !(options & 1) ) {
            O << _sExtraParameters << std::endl;
        }

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

        _bProcessing = name=="interpolation" || name=="hastimestamps" || name=="hasvelocities" || name=="pointtolerance" || name=="outputaccelchanges" || name=="multidofinterp"||name=="verifyinitialpath";
        return _bProcessing ? PE_Support : PE_Pass;
    }

    virtual bool endElement(const std::string& name)
    {
        if( _bProcessing ) {
            if( name == "interpolation") {
                _ss >> _interpolation;
            }
            else if( name == "hastimestamps" ) {
                _ss >> _hastimestamps;
            }
            else if( name == "hasvelocities" ) {
                _ss >> _hasvelocities;
            }
            else if( name == "pointtolerance" ) {
                _ss >> _pointtolerance;
            }
            else if( name == "outputaccelchanges" ) {
                _ss >> _outputaccelchanges;
            }
            else if( name == "multidofinterp" ) {
                _ss >> _multidofinterp;
            }
            else if( name == "verifyinitialpath") {
                _ss >> verifyinitialpath;
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

class OPENRAVE_API ConstraintTrajectoryTimingParameters : public TrajectoryTimingParameters
{
public:
    ConstraintTrajectoryTimingParameters() : TrajectoryTimingParameters(), maxlinkspeed(0), maxlinkaccel(0), maxmanipspeed(0), maxmanipaccel(0), vConstraintManipDir(0,0,1), vConstraintGlobalDir(0,0,1), fCosManipAngleThresh(-1), mingripperdistance(0), velocitydistancethresh(0), maxmergeiterations(1000), minswitchtime(0.2),nshortcutcycles(1), fSearchVelAccelMult(0.8), durationImprovementCutoffRatio(0.001), _bCProcessing(false) {
        _vXMLParameters.push_back("maxlinkspeed");
        _vXMLParameters.push_back("maxlinkaccel");
        _vXMLParameters.push_back("manipname");
        _vXMLParameters.push_back("maxmanipspeed");
        _vXMLParameters.push_back("maxmanipaccel");
        _vXMLParameters.push_back("constraintmanipdir");
        _vXMLParameters.push_back("constraintglobaldir");
        _vXMLParameters.push_back("cosmanipanglethresh");
        _vXMLParameters.push_back("mingripperdistance");
        _vXMLParameters.push_back("velocitydistancethresh");
        _vXMLParameters.push_back("maxmergeiterations");
        _vXMLParameters.push_back("minswitchtime");
        _vXMLParameters.push_back("nshortcutcycles");
        _vXMLParameters.push_back("searchvelaccelmult");
        _vXMLParameters.push_back("durationimprovementcutoffratio");
    }

    dReal maxlinkspeed; ///< max speed in m/s that any point on any link goes. 0 means no speed limit
    dReal maxlinkaccel; ///< max accel in m/s^2 that any point on the link goes. Gravity is always included in the acceleration computations. 0 means no accel limit

    std::string manipname; ///< the manip name to constrain the speed/accel of a manipulator to.
    dReal maxmanipspeed; ///< if non-zero then the timer shoulld consdier the max speed limit (m/s) of the active manipulators of the selected robots in the configuration space. 0 means no speed limit
    dReal maxmanipaccel; ///< if non-zero then the timer shoulld consdier the max acceleration limit (m/s^2) of the active manipulators of the selected robots in the configuration space. Gravity is always included in the acceleration computations. 0 means no accel limit

    Vector vConstraintManipDir, vConstraintGlobalDir; /// threshold the dot product between a direction on the manipulator and in the global world coordinate system. Use manipname
    dReal fCosManipAngleThresh; ///< cos of the angle threshold between vConstraintManipDir and vConstraintGlobalDir. dot(manipdir, manipdir) > cosmanipanglethresh. By default it is -1


    dReal mingripperdistance; ///< minimum distance of the hand (manipulator grippers) to any object. 0 means disabled.
    dReal velocitydistancethresh; /// threshold for dot(Direction,Velocity)/MinDistance where Direction is between the closest contact points. 0 if disabled.

    // merging waypoint related parameters
    int maxmergeiterations; ///< when merging several ramps together, the order that they are merged in depends. This parameters pecifies how many permutations to test before giving up.
    dReal minswitchtime; ///< the minimum time between switching accelerations of any joint (waypoints).
    int nshortcutcycles; ///< the minimum number of times the shortcut cycle is repeated.

    dReal fSearchVelAccelMult; ///< a number in [0.0001,0.99999] that is the multipler of the velocity/acceleration limits when time-based constraints are invalidated (manip speed and/or dynamics). The closer to 1 it is, the more optimal the trajectory will be, but it will take more time to compute. A value around 0.5-0.8 is best.
    dReal durationImprovementCutoffRatio; ///< Whenever shortcut is accepted, if change is less than diff/iterations, then do not do anymore shortcutting.

protected:
    bool _bCProcessing;
    virtual bool serialize(std::ostream& O, int options=0) const
    {
        if( !TrajectoryTimingParameters::serialize(O, options&~1) ) {
            return false;
        }
        O << "<maxlinkspeed>" << maxlinkspeed << "</maxlinkspeed>" << std::endl;
        O << "<maxlinkaccel>" << maxlinkaccel << "</maxlinkaccel>" << std::endl;
        O << "<manipname>" << manipname << "</manipname>" << std::endl;
        O << "<maxmanipspeed>" << maxmanipspeed << "</maxmanipspeed>" << std::endl;
        O << "<maxmanipaccel>" << maxmanipaccel << "</maxmanipaccel>" << std::endl;
        O << "<constraintmanipdir>" << vConstraintManipDir << "</constraintmanipdir>" << std::endl;
        O << "<constraintglobaldir>" << vConstraintGlobalDir << "</constraintglobaldir>" << std::endl;
        O << "<cosmanipanglethresh>" << fCosManipAngleThresh << "</cosmanipanglethresh>" << std::endl;
        O << "<mingripperdistance>" << mingripperdistance << "</mingripperdistance>" << std::endl;
        O << "<velocitydistancethresh>" << velocitydistancethresh << "</velocitydistancethresh>" << std::endl;
        O << "<maxmergeiterations>" << maxmergeiterations << "</maxmergeiterations>" << std::endl;
        O << "<minswitchtime>" << minswitchtime << "</minswitchtime>" << std::endl;
        O << "<nshortcutcycles>" << nshortcutcycles << "</nshortcutcycles>" << std::endl;
        O << "<searchvelaccelmult>" << fSearchVelAccelMult << "</searchvelaccelmult>" << std::endl;
        O << "<durationimprovementcutoffratio>" << durationImprovementCutoffRatio << "</durationimprovementcutoffratio>" << std::endl;
        if( !(options & 1) ) {
            O << _sExtraParameters << std::endl;
        }

        return !!O;
    }

    ProcessElement startElement(const std::string& name, const AttributesList& atts)
    {
        if( _bCProcessing ) {
            return PE_Ignore;
        }
        switch( TrajectoryTimingParameters::startElement(name,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }
        _bCProcessing = name=="maxlinkspeed" || name =="maxlinkaccel" || name=="manipname" || name=="maxmanipspeed" || name =="maxmanipaccel" || name=="mingripperdistance" || name=="velocitydistancethresh" || name=="maxmergeiterations" || name=="minswitchtime"|| name=="nshortcutcycles" || name=="constraintmanipdir" || name=="constraintglobaldir" || name=="cosmanipanglethresh" || name=="searchvelaccelmult" || name=="durationimprovementcutoffratio";
        return _bCProcessing ? PE_Support : PE_Pass;
    }

    virtual bool endElement(const std::string& name)
    {
        if( _bCProcessing ) {
            if( name == "maxlinkspeed") {
                _ss >> maxlinkspeed;
            }
            else if( name == "maxlinkaccel") {
                _ss >> maxlinkaccel;
            }
            else if( name == "manipname" ) {
                _ss >> manipname;
            }
            else if( name == "maxmanipspeed") {
                _ss >> maxmanipspeed;
            }
            else if( name == "maxmanipaccel") {
                _ss >> maxmanipaccel;
            }
            else if( name == "mingripperdistance" ) {
                _ss >> mingripperdistance;
            }
            else if( name == "velocitydistancethresh" ) {
                _ss >> velocitydistancethresh;
            }
            else if( name == "maxmergeiterations") {
                _ss >> maxmergeiterations;
            }
            else if( name == "minswitchtime") {
                _ss >> minswitchtime;
            }
            else if( name == "nshortcutcycles") {
                _ss >> nshortcutcycles;
            }
            else if( name == "searchvelaccelmult") {
                _ss >> fSearchVelAccelMult;
            }
            else if( name == "durationimprovementcutoffratio" ) {
                _ss >> durationImprovementCutoffRatio;
            }
            else if( name == "constraintmanipdir" ) {
                _ss >> vConstraintManipDir;
            }
            else if( name == "constraintglobaldir" ) {
                _ss >> vConstraintGlobalDir;
            }
            else if( name == "cosmanipanglethresh" ) {
                _ss >> fCosManipAngleThresh;
            }
            else {
                RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
            }
            _bCProcessing = false;
            return false;
        }

        // give a chance for the default parameters to get processed
        return TrajectoryTimingParameters::endElement(name);
    }
};

typedef boost::shared_ptr<ConstraintTrajectoryTimingParameters> ConstraintTrajectoryTimingParametersPtr;
typedef boost::shared_ptr<ConstraintTrajectoryTimingParameters const> ConstraintTrajectoryTimingParametersConstPtr;

class OPENRAVE_API WorkspaceTrajectoryParameters : public PlannerBase::PlannerParameters
{
public:
    WorkspaceTrajectoryParameters(EnvironmentBasePtr penv);
    inline EnvironmentBasePtr GetEnv() const {
        return _penv;
    }

    dReal maxdeviationangle;     ///< the maximum angle the next iksolution can deviate from the expected direction computed by the jacobian
    bool maintaintiming;     ///< maintain timing with input trajectory
    bool greedysearch;     ///< if true, will greeidly choose solutions (can possibly fail even a solution exists)
    dReal ignorefirstcollision;     ///< if > 0, will allow the robot to be in environment collision for the initial 'ignorefirstcollision' seconds of the trajectory. Once the robot gets out of collision, it will execute its normal following phase until it gets into collision again. This option is used when lifting objects from a surface, where the object is already in collision with the surface.
    dReal ignorefirstcollisionee;     ///< if > 0, will allow the manipulator end effector to be in environment collision for the initial 'ignorefirstcollisionee' seconds of the trajectory. similar to 'ignorefirstcollision'
    dReal ignorelastcollisionee; /// if > 0, will allow the manipulator end effector to get into collision with the environment for the last 'ignorelastcollisionee' seconds of the trajrectory. The kinematics, self collisions, and environment collisions with the other parts of the robot will still be checked
    dReal minimumcompletetime;     ///< specifies the minimum trajectory that must be followed for planner to declare success. If 0, then the entire trajectory has to be followed.
    TrajectoryBasePtr workspacetraj;     ///< workspace trajectory

protected:
    EnvironmentBasePtr _penv;
    BaseXMLReaderPtr _pcurreader;
    bool _bProcessing;

    virtual bool serialize(std::ostream& O, int options=0) const;
    virtual ProcessElement startElement(const std::string& name, const AttributesList& atts);
    virtual bool endElement(const std::string& name);
    virtual void characters(const std::string& ch);
};

typedef boost::shared_ptr<WorkspaceTrajectoryParameters> WorkspaceTrajectoryParametersPtr;
typedef boost::shared_ptr<WorkspaceTrajectoryParameters const> WorkspaceTrajectoryParametersConstPtr;

class OPENRAVE_API RRTParameters : public PlannerBase::PlannerParameters
{
public:
    RRTParameters() : _minimumgoalpaths(1), _bProcessing(false) {
        _vXMLParameters.push_back("minimumgoalpaths");
    }

    size_t _minimumgoalpaths; ///< minimum number of goals to connect to before exiting. the goal with the shortest path is returned.

protected:
    bool _bProcessing;
    virtual bool serialize(std::ostream& O, int options=0) const
    {
        if( !PlannerParameters::serialize(O, options&~1) ) {
            return false;
        }
        O << "<minimumgoalpaths>" << _minimumgoalpaths << "</minimumgoalpaths>" << std::endl;
        if( !(options & 1) ) {
            O << _sExtraParameters << std::endl;
        }
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

        _bProcessing = name=="minimumgoalpaths";
        return _bProcessing ? PE_Support : PE_Pass;
    }

    virtual bool endElement(const std::string& name)
    {
        if( _bProcessing ) {
            if( name == "minimumgoalpaths") {
                _ss >> _minimumgoalpaths;
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

typedef boost::shared_ptr<RRTParameters> RRTParametersPtr;

class OPENRAVE_API BasicRRTParameters : public RRTParameters
{
public:
    BasicRRTParameters() : RRTParameters(), _fGoalBiasProb(0.05f), _nRRTExtentType(0), _nMinIterations(0), _bProcessingBasic(false) {
        _vXMLParameters.push_back("goalbias");
        _vXMLParameters.push_back("nrrtextenttype");
        _vXMLParameters.push_back("nminiterations");
    }

    dReal _fGoalBiasProb;
    int _nRRTExtentType; ///< the rrt extent type. if 0 then extend all the way to the sampled position. if 1, then just extend one step length before sampling again.
    int _nMinIterations; ///< minimum iterations to do before returning a result unless user passes a PA_ReturnWithAnySolution interrupt. When the iterations are done, the RRT will choose the goal with the minimum configuration distance from the initial to return. By defautl it is 0.

protected:
    bool _bProcessingBasic;
    virtual bool serialize(std::ostream& O, int options=0) const
    {
        if( !RRTParameters::serialize(O, options&~1) ) {
            return false;
        }
        O << "<goalbias>" << _fGoalBiasProb << "</goalbias>" << std::endl;
        O << "<nrrtextenttype>" << _nRRTExtentType << "</nrrtextenttype>" << std::endl;
        O << "<nminiterations>" << _nMinIterations << "</nminiterations>" << std::endl;
        if( !(options & 1) ) {
            O << _sExtraParameters << std::endl;
        }

        return !!O;
    }

    ProcessElement startElement(const std::string& name, const AttributesList& atts)
    {
        if( _bProcessingBasic ) {
            return PE_Ignore;
        }
        switch( RRTParameters::startElement(name,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }

        _bProcessingBasic = name=="goalbias" || name =="nrrtextenttype" || name=="nminiterations";
        return _bProcessingBasic ? PE_Support : PE_Pass;
    }

    virtual bool endElement(const std::string& name)
    {
        if( _bProcessingBasic ) {
            if( name == "goalbias") {
                _ss >> _fGoalBiasProb;
            }
            else if( name == "nrrtextenttype" ) {
                _ss >> _nRRTExtentType;
            }
            else if( name == "nminiterations" ) {
                _ss >> _nMinIterations;
            }
            else {
                RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
            }
            _bProcessingBasic = false;
            return false;
        }

        // give a chance for the default parameters to get processed
        return RRTParameters::endElement(name);
    }
};

typedef boost::shared_ptr<BasicRRTParameters> BasicRRTParametersPtr;

} // OpenRAVE

#endif
