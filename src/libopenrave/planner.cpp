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
#include "libopenrave.h"

#include <openrave/planningutils.h>

namespace OpenRAVE {

std::istream& operator>>(std::istream& I, PlannerBase::PlannerParameters& pp)
{
    if( !!I) {
        stringbuf buf;
        stringstream::streampos pos = I.tellg();
        I.get(buf, 0); // get all the data, yes this is inefficient, not sure if there anyway to search in streams

        string pbuf = buf.str();
        const char* p = strcasestr(pbuf.c_str(), "</PlannerParameters>");
        int ppsize=-1;
        if( p != NULL ) {
            I.clear();
            ppsize=(p-pbuf.c_str())+20;
            I.seekg((size_t)pos+ppsize);
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT("error, failed to find </PlannerParameters> in %s",buf.str(),ORE_InvalidArguments);
        }
        pp._plannerparametersdepth = 0;
        LocalXML::ParseXMLData(PlannerBase::PlannerParametersPtr(&pp,null_deleter()), pbuf.c_str(), ppsize);
    }

    return I;
}

void subtractstates(std::vector<dReal>& q1, const std::vector<dReal>& q2)
{
    BOOST_ASSERT(q1.size()==q2.size());
    for(size_t i = 0; i < q1.size(); ++i) {
        q1[i] -= q2[i];
    }
}

bool addstates(std::vector<dReal>& q, const std::vector<dReal>& qdelta, int fromgoal)
{
    BOOST_ASSERT(q.size()==qdelta.size());
    for(size_t i = 0; i < q.size(); ++i) {
        q[i] += qdelta[i];
    }
    return true;
}

PlannerBase::PlannerParameters::PlannerParameters() : XMLReadable("plannerparameters"), _fStepLength(0.04f), _nMaxIterations(0), _sPostProcessingPlanner("shortcut_linear")
{
    _diffstatefn = subtractstates;
    _neighstatefn = addstates;
    //_sPostProcessingParameters ="<_nmaxiterations>100</_nmaxiterations><_postprocessing planner=\"lineartrajectoryretimer\"></_postprocessing>";
    _sPostProcessingParameters ="<_nmaxiterations>20</_nmaxiterations><_postprocessing planner=\"parabolicsmoother\"><_nmaxiterations>200</_nmaxiterations></_postprocessing>";
    _vXMLParameters.reserve(20);
    _vXMLParameters.push_back("configuration");
    _vXMLParameters.push_back("_vinitialconfig");
    _vXMLParameters.push_back("_vgoalconfig");
    _vXMLParameters.push_back("_vconfiglowerlimit");
    _vXMLParameters.push_back("_vconfigupperlimit");
    _vXMLParameters.push_back("_vconfigvelocitylimit");
    _vXMLParameters.push_back("_vconfigaccelerationlimit");
    _vXMLParameters.push_back("_vconfigresolution");
    _vXMLParameters.push_back("_nmaxiterations");
    _vXMLParameters.push_back("_fsteplength");
    _vXMLParameters.push_back("_postprocessing");
}

PlannerBase::PlannerParameters::PlannerParameters(const PlannerParameters &r) : XMLReadable("")
{
    BOOST_ASSERT(0);
}

PlannerBase::PlannerParameters& PlannerBase::PlannerParameters::operator=(const PlannerBase::PlannerParameters& r)
{
    // reset
    _costfn = r._costfn;
    _goalfn = r._goalfn;
    _distmetricfn = r._distmetricfn;
    _checkpathconstraintsfn = r._checkpathconstraintsfn;
    _samplefn = r._samplefn;
    _sampleneighfn = r._sampleneighfn;
    _samplegoalfn = r._samplegoalfn;
    _sampleinitialfn = r._sampleinitialfn;
    _setstatefn = r._setstatefn;
    _getstatefn = r._getstatefn;
    _diffstatefn = r._diffstatefn;
    _neighstatefn = r._neighstatefn;

    vinitialconfig.resize(0);
    vgoalconfig.resize(0);
    _configurationspecification = ConfigurationSpecification();
    _vConfigLowerLimit.resize(0);
    _vConfigUpperLimit.resize(0);
    _vConfigResolution.resize(0);
    _vConfigVelocityLimit.resize(0);
    _vConfigAccelerationLimit.resize(0);
    _sPostProcessingPlanner = "";
    _sPostProcessingParameters.resize(0);
    _sExtraParameters.resize(0);
    _nMaxIterations = 0;
    _fStepLength = 0.04f;
    _plannerparametersdepth = 0;

    // transfer data
    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits<dReal>::digits10+1); /// have to do this or otherwise precision gets lost and planners' initial conditions can vioalte constraints
    ss << r;
    ss >> *this;
    return *this;
}

void PlannerBase::PlannerParameters::copy(boost::shared_ptr<PlannerParameters const> r)
{
    *this = *r;
}

bool PlannerBase::PlannerParameters::serialize(std::ostream& O) const
{
    O << _configurationspecification << endl;
    O << "<_vinitialconfig>";
    FOREACHC(it, vinitialconfig) {
        O << *it << " ";
    }
    O << "</_vinitialconfig>" << endl;
    O << "<_vgoalconfig>";
    FOREACHC(it, vgoalconfig) {
        O << *it << " ";
    }
    O << "</_vgoalconfig>" << endl;
    O << "<_vconfiglowerlimit>";
    FOREACHC(it, _vConfigLowerLimit) {
        O << *it << " ";
    }
    O << "</_vconfiglowerlimit>" << endl;
    O << "<_vconfigupperlimit>";
    FOREACHC(it, _vConfigUpperLimit) {
        O << *it << " ";
    }
    O << "</_vconfigupperlimit>" << endl;
    O << "<_vconfigvelocitylimit>";
    FOREACHC(it, _vConfigVelocityLimit) {
        O << *it << " ";
    }
    O << "</_vconfigvelocitylimit>" << endl;
    O << "<_vconfigaccelerationlimit>";
    FOREACHC(it, _vConfigAccelerationLimit) {
        O << *it << " ";
    }
    O << "</_vconfigaccelerationlimit>" << endl;
    O << "<_vconfigresolution>";
    FOREACHC(it, _vConfigResolution) {
        O << *it << " ";
    }
    O << "</_vconfigresolution>" << endl;

    O << "<_nmaxiterations>" << _nMaxIterations << "</_nmaxiterations>" << endl;
    O << "<_fsteplength>" << _fStepLength << "</_fsteplength>" << endl;
    O << "<_postprocessing planner=\"" << _sPostProcessingPlanner << "\">" << _sPostProcessingParameters << "</_postprocessing>" << endl;
    O << _sExtraParameters << endl;
    return !!O;
}

BaseXMLReader::ProcessElement PlannerBase::PlannerParameters::startElement(const std::string& name, const AttributesList& atts)
{
    _ss.str(""); // have to clear the string
    if( !!__pcurreader ) {
        if( __pcurreader->startElement(name, atts) == PE_Support ) {
            return PE_Support;
        }
        return PE_Ignore;
    }

    if( __processingtag.size() > 0 ) {
        return PE_Ignore;
    }
    if( name=="plannerparameters" ) {
        _plannerparametersdepth++;
        return PE_Support;
    }

    if( name == "_postprocessing" ) {
        _sslocal.reset(new std::stringstream());
        _sPostProcessingPlanner="";
        _sPostProcessingParameters="";
        FOREACHC(itatt,atts) {
            if( itatt->first == "planner" ) {
                _sPostProcessingPlanner = itatt->second;
            }
        }
        __pcurreader.reset(new DummyXMLReader(name,GetXMLId(),_sslocal));
        return PE_Support;
    }

    if( find(_vXMLParameters.begin(),_vXMLParameters.end(),name) == _vXMLParameters.end() ) {
        _sslocal.reset(new std::stringstream());
        *_sslocal << "<" << name << " ";
        FOREACHC(itatt, atts) {
            *_sslocal << itatt->first << "=\"" << itatt->second << "\" ";
        }
        *_sslocal << ">" << endl;
        __pcurreader.reset(new DummyXMLReader(name,GetXMLId(),_sslocal));
        return PE_Support;
    }

    if( name == "configuration" ) {
        __pcurreader.reset(new ConfigurationSpecification::Reader(_configurationspecification));
        return PE_Support;
    }

    static const boost::array<std::string,10> names = {{"_vinitialconfig","_vgoalconfig","_vconfiglowerlimit","_vconfigupperlimit","_vconfigvelocitylimit","_vconfigaccelerationlimit","_vconfigresolution","_nmaxiterations","_fsteplength","_postprocessing"}};
    if( find(names.begin(),names.end(),name) != names.end() ) {
        __processingtag = name;
        return PE_Support;
    }
    return PE_Pass;
}

bool PlannerBase::PlannerParameters::endElement(const std::string& name)
{
    if( !!__pcurreader ) {
        if( __pcurreader->endElement(name) ) {
            boost::shared_ptr<DummyXMLReader> pdummy = boost::dynamic_pointer_cast<DummyXMLReader>(__pcurreader);
            if( !!pdummy ) {
                if( pdummy->GetFieldName() == "_postprocessing" ) {
                    _sPostProcessingParameters = _sslocal->str();
                    _sslocal.reset();
                }
                else {
                    *_sslocal << "</" << name << ">" << endl;
                    _sExtraParameters += _sslocal->str();
                    _sslocal.reset();
                }
            }
            __pcurreader.reset();
        }
    }
    else if( name == "plannerparameters" ) {
        return --_plannerparametersdepth < 0;
    }
    else if( __processingtag.size() > 0 ) {
        if( name == "_vinitialconfig") {
            vinitialconfig = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        }
        else if( name == "_vgoalconfig") {
            vgoalconfig = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        }
        else if( name == "_vconfiglowerlimit") {
            _vConfigLowerLimit = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        }
        else if( name == "_vconfigupperlimit") {
            _vConfigUpperLimit = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        }
        else if( name == "_vconfigresolution") {
            _vConfigResolution = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        }
        else if( name == "_vconfigvelocitylimit") {
            _vConfigVelocityLimit = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        }
        else if( name == "_vconfigaccelerationlimit") {
            _vConfigAccelerationLimit = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        }
        else if( name == "_nmaxiterations") {
            _ss >> _nMaxIterations;
        }
        else if( name == "_fsteplength") {
            _ss >> _fStepLength;
        }
        if( name !=__processingtag ) {
            RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%name%__processingtag));
        }
        __processingtag = "";
        return false;
    }

    return false;
}

void PlannerBase::PlannerParameters::characters(const std::string& ch)
{
    if( !!__pcurreader ) {
        __pcurreader->characters(ch);
    }
    else {
        _ss.clear();
        _ss << ch;
    }
}

std::ostream& operator<<(std::ostream& O, const PlannerBase::PlannerParameters& v)
{
    O << "<" << v.GetXMLId() << ">" << endl;
    v.serialize(O);
    O << "</" << v.GetXMLId() << ">" << endl;
    return O;
}

void PlannerBase::PlannerParameters::SetRobotActiveJoints(RobotBasePtr robot)
{
    using namespace planningutils;
    _distmetricfn = boost::bind(&SimpleDistanceMetric::Eval,boost::shared_ptr<SimpleDistanceMetric>(new SimpleDistanceMetric(robot)),_1,_2);
    SpaceSamplerBasePtr pconfigsampler = RaveCreateSpaceSampler(robot->GetEnv(),str(boost::format("robotconfiguration %s")%robot->GetName()));
    boost::shared_ptr<SimpleNeighborhoodSampler> defaultsamplefn(new SimpleNeighborhoodSampler(pconfigsampler,_distmetricfn));
    _samplefn = boost::bind(&SimpleNeighborhoodSampler::Sample,defaultsamplefn,_1);
    _sampleneighfn = boost::bind(&SimpleNeighborhoodSampler::Sample,defaultsamplefn,_1,_2,_3);
    _setstatefn = boost::bind(&RobotBase::SetActiveDOFValues,robot,_1,false);
    _getstatefn = boost::bind(&RobotBase::GetActiveDOFValues,robot,_1);
    _diffstatefn = boost::bind(&RobotBase::SubtractActiveDOFValues,robot,_1,_2);
    _neighstatefn = addstates; // probably ok...

    boost::shared_ptr<LineCollisionConstraint> pcollision(new LineCollisionConstraint());
    _checkpathconstraintsfn = boost::bind(&LineCollisionConstraint::Check,pcollision,PlannerParametersWeakPtr(shared_parameters()), robot, _1, _2, _3, _4);

    robot->GetActiveDOFLimits(_vConfigLowerLimit,_vConfigUpperLimit);
    robot->GetActiveDOFVelocityLimits(_vConfigVelocityLimit);
    robot->GetActiveDOFAccelerationLimits(_vConfigAccelerationLimit);
    robot->GetActiveDOFResolutions(_vConfigResolution);
    robot->GetActiveDOFValues(vinitialconfig);
    _configurationspecification = robot->GetActiveConfigurationSpecification();
}

PlannerBase::PlannerProgress::PlannerProgress() : _iteration(0)
{
}

class CustomPlannerCallbackData : public boost::enable_shared_from_this<CustomPlannerCallbackData>, public UserData
{
public:
    CustomPlannerCallbackData(const PlannerBase::PlanCallbackFn& callbackfn, PlannerBasePtr planner) : _callbackfn(callbackfn), _plannerweak(planner) {
    }
    virtual ~CustomPlannerCallbackData() {
        PlannerBasePtr planner = _plannerweak.lock();
        if( !!planner ) {
            planner->__listRegisteredCallbacks.erase(_iterator);
        }
    }

    PlannerBase::PlanCallbackFn _callbackfn;
    PlannerBaseWeakPtr _plannerweak;
    std::list<UserDataWeakPtr>::iterator _iterator;
};

typedef boost::shared_ptr<CustomPlannerCallbackData> CustomPlannerCallbackDataPtr;

PlannerBase::PlannerBase(EnvironmentBasePtr penv) : InterfaceBase(PT_Planner, penv)
{
}

bool PlannerBase::InitPlan(RobotBasePtr pbase, std::istream& isParameters)
{
    RAVELOG_WARN(str(boost::format("using default planner parameters structure to de-serialize parameters data inside %s, information might be lost!! Please define a InitPlan(robot,stream) function!\n")%GetXMLId()));
    boost::shared_ptr<PlannerParameters> localparams(new PlannerParameters());
    isParameters >> *localparams;
    return InitPlan(pbase,localparams);
}

UserDataPtr PlannerBase::RegisterPlanCallback(const PlanCallbackFn& callbackfn)
{
    CustomPlannerCallbackDataPtr pdata(new CustomPlannerCallbackData(callbackfn,shared_planner()));
    pdata->_iterator = __listRegisteredCallbacks.insert(__listRegisteredCallbacks.end(),pdata);
    return pdata;
}

PlannerStatus PlannerBase::_ProcessPostPlanners(RobotBasePtr probot, TrajectoryBasePtr ptraj)
{
    if( GetParameters()->_sPostProcessingPlanner.size() == 0 ) {
        return PS_HasSolution;
    }
    PlannerBasePtr planner = RaveCreatePlanner(GetEnv(), GetParameters()->_sPostProcessingPlanner);
    if( !planner ) {
        planner = RaveCreatePlanner(GetEnv(), "shortcut_linear");
        if( !planner ) {
            return PS_Failed;
        }
    }
    PlannerParametersPtr params(new PlannerParameters());
    params->copy(GetParameters());
    params->_sExtraParameters += GetParameters()->_sPostProcessingParameters;
    params->_sPostProcessingPlanner = "";
    params->_sPostProcessingParameters = "";
    params->_nMaxIterations = 0; // have to reset since path optimizers also use it and new parameters could be in extra parameters
    if( planner->InitPlan(probot, params) ) {
        PlannerStatus status = planner->PlanPath(ptraj);
        if( status != PS_Failed ) {
            return status;
        }
    }

    if( planner->GetXMLId() != "shortcut_linear") {
        planner = RaveCreatePlanner(GetEnv(), "shortcut_linear");
        if( !!planner ) {
            RAVELOG_WARN(str(boost::format("%s post processing failed, trying shortcut_linear")%GetParameters()->_sPostProcessingPlanner));
            if( planner->InitPlan(probot, params) ) {
                PlannerStatus status = planner->PlanPath(ptraj);
                if( status != PS_Failed ) {
                    return status;
                }
            }
        }
    }
    return PS_Failed;
}

PlannerAction PlannerBase::_CallCallbacks(const PlannerProgress& progress)
{
    FOREACHC(it,__listRegisteredCallbacks) {
        CustomPlannerCallbackDataPtr pitdata = boost::dynamic_pointer_cast<CustomPlannerCallbackData>(it->lock());
        if( !!pitdata) {
            PlannerAction ret = pitdata->_callbackfn(progress);
            if( ret != PA_None ) {
                return ret;
            }
        }
    }
    return PA_None;
}

}
