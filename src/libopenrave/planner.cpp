// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov (rosen.diankov@gmail.com)
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
#include <openrave/xmlreaders.h>

namespace OpenRAVE {

static std::string s_linearsmoother = "linearsmoother"; //"shortcut_linear";

std::istream& operator>>(std::istream& I, PlannerParameters& pp)
{
    if( !!I) {
        stringbuf buf;

        //std::istream::sentry sentry(I); // necessary?!
        stringstream::streampos pos = I.tellg();
        I.seekg(0, ios::end);
        stringstream::streampos endpos = I.tellg();
        I.seekg(pos);

        std::vector<char> vstrbuf; vstrbuf.reserve((size_t)(endpos-pos)); // make sure there are at least this many bytes

        const char* pMatchPlannerParameters = "</PlannerParameters>";
        size_t nMatchPlannerParametersSize = 20;
        bool bFoundMatch = false;
        for (char c = I.get(); I; c = I.get()) {
            vstrbuf.push_back(c);
            if( c == pMatchPlannerParameters[nMatchPlannerParametersSize-1] ) {
                // matches at the end, check if previous string matches
                if( vstrbuf.size() >= nMatchPlannerParametersSize) {
                    if( strncasecmp(&vstrbuf[vstrbuf.size()-nMatchPlannerParametersSize], pMatchPlannerParameters, nMatchPlannerParametersSize) == 0 ) {
                        // matches, so quit
                        bFoundMatch = true;
                        break;
                    }
                }
            }
        }

        if( !bFoundMatch ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("error, failed to find </PlannerParameters> in %s"),buf.str(),ORE_InvalidArguments);
        }

        pp._plannerparametersdepth = 0;
        xmlreaders::ParseXMLData(pp, &vstrbuf[0], vstrbuf.size());
    }

    return I;
}

void SubtractStates(std::vector<dReal>& q1, const std::vector<dReal>& q2)
{
    BOOST_ASSERT(q1.size()==q2.size());
    for(size_t i = 0; i < q1.size(); ++i) {
        q1[i] -= q2[i];
    }
}

int AddStates(std::vector<dReal>& q, const std::vector<dReal>& qdelta, int fromgoal)
{
    BOOST_ASSERT(q.size()==qdelta.size());
    for(size_t i = 0; i < q.size(); ++i) {
        q[i] += qdelta[i];
    }
    return NSS_Reached;
}

int AddStatesWithLimitCheck(std::vector<dReal>& q, const std::vector<dReal>& qdelta, int fromgoal, const std::vector<dReal>& vLowerLimits, const std::vector<dReal>& vUpperLimits)
{
    BOOST_ASSERT(q.size()==qdelta.size());
    int status = NSS_Reached;
    for(size_t i = 0; i < q.size(); ++i) {
        q[i] += qdelta[i];
        if( q[i] > vUpperLimits.at(i) ) {
            q[i] = vUpperLimits.at(i);
            status |= 0x2;
        }
        else if( q[i] < vLowerLimits.at(i) ) {
            q[i] = vLowerLimits.at(i);
            status |= 0x2;
        }
    }
    return status;
}

PlannerStatus::PlannerStatus()
{
    statusCode = 0;
    numPlannerIterations = 0;
    elapsedPlanningTimeUS = 0;
}

PlannerStatus::PlannerStatus(const uint32_t statusCode) : PlannerStatus()
{
    this->statusCode = statusCode;
    if(statusCode & PS_HasSolution) {
        description += "Planner succeeded. ";
    }
    else {
        description += "Planner failed with generic error. ";
    }

    if(statusCode & PS_Interrupted) {
        description += "Planning was interrupted, but can be resumed by calling PlanPath again. ";
    }
    if(statusCode & PS_InterruptedWithSolution) {
        description += "Planning was interrupted, but a valid path/solution was returned. Can call PlanPath again to refine results. ";
    }
    if(statusCode & PS_FailedDueToCollision) {
        description += "planner failed due to collision constraints. ";
    }
    if(statusCode & PS_FailedDueToInitial) {
        description += "Failed due to initial configurations. ";
    }
    if(statusCode & PS_FailedDueToGoal) {
        description += "Failed due to goal configurations. ";
    }
    if(statusCode & PS_FailedDueToKinematics) {
        description += "Failed due to kinematics constraints. ";
    }
    if(statusCode & PS_FailedDueToIK) {
        description += "Failed due to inverse kinematics (could be due to collisions or velocity constraints, but don't know). ";
    }
    if(statusCode & PS_FailedDueToVelocityConstraints) {
        description += "Failed due to velocity constraints. ";
    }

    if(description.empty()) {
        RAVELOG_WARN_FORMAT(_("planner status code (0x%x) is not supported by planner status default constructor"), statusCode);
    }
}

PlannerStatus::PlannerStatus(const std::string& newdescription, const uint32_t statusCode) :
    PlannerStatus(statusCode)
{
    if( description.empty() ) {
        description = newdescription;
    }
    else {
        description = newdescription + std::string(" ") + description;
    }

}

PlannerStatus::PlannerStatus(const std::string& newdescription, const uint32_t statusCode, CollisionReportPtr& report) :
    PlannerStatus(newdescription, statusCode)
{
    InitCollisionReport(report);
}

PlannerStatus::PlannerStatus(const std::string& description, const uint32_t statusCode, const IkParameterization& ikparam) :
    PlannerStatus(description, statusCode)
{
    this->ikparam = ikparam;
}

PlannerStatus::PlannerStatus(const std::string& description, const uint32_t statusCode, const IkParameterization& ikparam, CollisionReportPtr& report) :
    PlannerStatus(description, statusCode, ikparam)
{
    InitCollisionReport(report);
}

PlannerStatus::PlannerStatus(const std::string& description, const uint32_t statusCode, const std::vector<dReal>& jointValues) :
    PlannerStatus(description, statusCode)
{
    this->jointValues = jointValues;
}

PlannerStatus::PlannerStatus(const std::string& description, const uint32_t statusCode, const std::vector<dReal>& jointValues, CollisionReportPtr& report) :
    PlannerStatus(description, statusCode, jointValues)
{
    InitCollisionReport(report);
}

PlannerStatus::~PlannerStatus() {
}

PlannerStatus& PlannerStatus::SetErrorOrigin(const std::string& errorOrigin)
{
    this->errorOrigin = errorOrigin;
    return *this;
}

PlannerStatus& PlannerStatus::SetPlannerParameters(PlannerParametersConstPtr parameters)
{
    this->parameters = parameters;
    return *this;
}

void PlannerStatus::InitCollisionReport(CollisionReportPtr& newreport)
{
    if( !!newreport ) {
        // should copy
        if( !report ) {
            report.reset(new CollisionReport());
        }
        *report = *newreport;
    }
    else {
        report.reset();
    }
}

void PlannerStatus::AddCollisionReport(const CollisionReport& collisionReport)
{
    if (!!collisionReport.plink1 && !!collisionReport.plink2) {
        std::pair<KinBody::LinkConstPtr,KinBody::LinkConstPtr> collisionPair(collisionReport.plink1, collisionReport.plink2);
        std::map<std::pair<KinBody::LinkConstPtr,KinBody::LinkConstPtr>, unsigned int>::iterator collideLinkPairKey = mCollidingLinksCount.find(collisionPair);
        if (collideLinkPairKey != mCollidingLinksCount.end()) {
            collideLinkPairKey->second += 1;
        } else {
            mCollidingLinksCount[collisionPair] = 1;
        }
    }
}

void PlannerStatus::SaveToJson(rapidjson::Value& rPlannerStatus, rapidjson::Document::AllocatorType& alloc) const
{
    rPlannerStatus.SetObject();
    orjson::SetJsonValueByKey(rPlannerStatus, "errorOrigin", errorOrigin, alloc);
    orjson::SetJsonValueByKey(rPlannerStatus, "description", description, alloc);
    orjson::SetJsonValueByKey(rPlannerStatus, "statusCode", statusCode, alloc);
    if(jointValues.size() > 0) {
        orjson::SetJsonValueByKey(rPlannerStatus, "jointValues", jointValues, alloc);
    }

    if(!!report) {
        rapidjson::Value reportjson(rapidjson::kObjectType);
        if(!!report->plink1) {
            orjson::SetJsonValueByKey(reportjson, "plink1", report->plink1->GetName(), alloc);
        }
        if(!!report->plink2) {
            orjson::SetJsonValueByKey(reportjson, "plink2", report->plink2->GetName(), alloc);
        }
        rapidjson::Value reportContactsjson(rapidjson::kObjectType);
        for (size_t i=0; i<report->contacts.size(); ++i) {
            rapidjson::Value reportContactsPosjson(rapidjson::kObjectType);
            orjson::SetJsonValueByKey(reportContactsPosjson, "x", report->contacts[i].pos.x, alloc);
            orjson::SetJsonValueByKey(reportContactsPosjson, "y", report->contacts[i].pos.y, alloc);
            orjson::SetJsonValueByKey(reportContactsPosjson, "z", report->contacts[i].pos.z, alloc);

            rapidjson::Value rname;
            orjson::SaveJsonValue(rname, std::to_string(i), alloc);
            reportContactsjson.AddMember(rname, reportContactsPosjson, alloc);
        }
        orjson::SetJsonValueByKey(reportjson, "contacts", reportContactsjson, alloc);
        //Eventually, serialization could be in openravejson.h
        orjson::SetJsonValueByKey(rPlannerStatus, "collisionReport", reportjson, alloc);
    }

    //Eventually, serialization could be in openravejson.h ?
    if( ikparam.GetType() != IKP_None ) {
        orjson::SetJsonValueByKey(rPlannerStatus, "ikparam", ikparam, alloc);
    }
}

PlannerParameters::StateSaver::StateSaver(PlannerParametersPtr params) : _params(params)
{
    _params->_getstatefn(_values);
    OPENRAVE_ASSERT_OP((int)_values.size(),==,_params->GetDOF());
}

PlannerParameters::StateSaver::~StateSaver()
{
    _Restore();
}

void PlannerParameters::StateSaver::Restore()
{
    _Restore();
}

void PlannerParameters::StateSaver::_Restore()
{
    int ret = _params->SetStateValues(_values, 0);
    BOOST_ASSERT(ret==0);
}

PlannerParameters::PlannerParameters() : Readable("plannerparameters"), _fStepLength(0.04f), _nMaxIterations(0), _nMaxPlanningTime(0), _sPostProcessingPlanner(s_linearsmoother), _nRandomGeneratorSeed(0)
{
    _diffstatefn = SubtractStates;
    _neighstatefn = AddStates;

    //_sPostProcessingParameters ="<_nmaxiterations>100</_nmaxiterations><_postprocessing planner=\"lineartrajectoryretimer\"></_postprocessing>";
    // should not verify initial path since coming from RRT. actually the linear smoother sometimes introduces small collisions due to the discrete nature of the collision checking, so also want to ignore those
    _sPostProcessingParameters ="<_nmaxiterations>20</_nmaxiterations><_postprocessing planner=\"parabolicsmoother\"><_nmaxiterations>100</_nmaxiterations><verifyinitialpath>0</verifyinitialpath></_postprocessing>";
    _vXMLParameters.reserve(20);
    _vXMLParameters.push_back("configuration");
    _vXMLParameters.push_back("_vinitialconfig");
    _vXMLParameters.push_back("_vinitialconfigvelocities");
    _vXMLParameters.push_back("_vgoalconfigvelocities");
    _vXMLParameters.push_back("_vgoalconfig");
    _vXMLParameters.push_back("_vconfiglowerlimit");
    _vXMLParameters.push_back("_vconfigupperlimit");
    _vXMLParameters.push_back("_vconfigvelocitylimit");
    _vXMLParameters.push_back("_vconfigaccelerationlimit");
    _vXMLParameters.push_back("_vconfigresolution");
    _vXMLParameters.push_back("_nmaxiterations");
    _vXMLParameters.push_back("_nmaxplanningtime");
    _vXMLParameters.push_back("_fsteplength");
    _vXMLParameters.push_back("_postprocessing");
    _vXMLParameters.push_back("_nrandomgeneratorseed");
}

PlannerParameters::~PlannerParameters()
{
}

PlannerParameters::PlannerParameters(const PlannerParameters &r)
{
}

PlannerParameters& PlannerParameters::operator=(const PlannerParameters& r)
{
    // reset
    _costfn = r._costfn;
    _goalfn = r._goalfn;
    _distmetricfn = r._distmetricfn;
    _checkpathvelocityconstraintsfn = r._checkpathvelocityconstraintsfn;
    _samplefn = r._samplefn;
    _sampleneighfn = r._sampleneighfn;
    _samplegoalfn = r._samplegoalfn;
    _sampleinitialfn = r._sampleinitialfn;
    _setstatevaluesfn = r._setstatevaluesfn;
    _getstatefn = r._getstatefn;
    _diffstatefn = r._diffstatefn;
    _neighstatefn = r._neighstatefn;
    _listInternalSamplers = r._listInternalSamplers;

    vinitialconfig.resize(0);
    _vInitialConfigVelocities.resize(0);
    _vGoalConfigVelocities.resize(0);
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
    _nMaxPlanningTime = 0;
    _fStepLength = 0.04f;
    _nRandomGeneratorSeed = 0;
    _plannerparametersdepth = 0;

    // transfer data
    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits<dReal>::digits10+1); /// have to do this or otherwise precision gets lost and planners' initial conditions can vioalte constraints
    ss << r;
    ss >> *this;
    return *this;
}

void PlannerParameters::copy(boost::shared_ptr<PlannerParameters const> r)
{
    *this = *r;
}

int PlannerParameters::SetStateValues(const std::vector<dReal>& values, int options) const
{
    if( !!_setstatevaluesfn ) {
        return _setstatevaluesfn(values, options);
    }
    throw openrave_exception(_("need to set PlannerParameters::_setstatevaluesfn"));
}

bool PlannerParameters::serialize(std::ostream& O, int options) const
{
    O << _configurationspecification << endl;
    O << "<_vinitialconfig>";
    FOREACHC(it, vinitialconfig) {
        O << *it << " ";
    }
    O << "</_vinitialconfig>" << endl;
    O << "<_vinitialconfigvelocities>";
    FOREACHC(it, _vInitialConfigVelocities) {
        O << *it << " ";
    }
    O << "</_vinitialconfigvelocities>" << endl;
    O << "<_vgoalconfig>";
    FOREACHC(it, vgoalconfig) {
        O << *it << " ";
    }
    O << "</_vgoalconfig>" << endl;
    O << "<_vgoalconfigvelocities>";
    FOREACHC(it, _vGoalConfigVelocities) {
        O << *it << " ";
    }
    O << "</_vgoalconfigvelocities>" << endl;
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
    O << "<_nmaxplanningtime>" << _nMaxPlanningTime << "</_nmaxplanningtime>" << endl;
    O << "<_fsteplength>" << _fStepLength << "</_fsteplength>" << endl;
    O << "<_nrandomgeneratorseed>" << _nRandomGeneratorSeed << "</_nrandomgeneratorseed>" << endl;
    O << "<_postprocessing planner=\"" << _sPostProcessingPlanner << "\">" << _sPostProcessingParameters << "</_postprocessing>" << endl;
    if( !(options & 1) ) {
        O << _sExtraParameters << endl;
    }
    return !!O;
}

BaseXMLReader::ProcessElement PlannerParameters::startElement(const std::string& name, const AttributesList& atts)
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

    static const boost::array<std::string,14> names = {{"_vinitialconfig","_vgoalconfig","_vconfiglowerlimit","_vconfigupperlimit","_vconfigvelocitylimit","_vconfigaccelerationlimit","_vconfigresolution","_nmaxiterations","_nmaxplanningtime","_fsteplength","_postprocessing", "_nrandomgeneratorseed", "_vinitialconfigvelocities", "_vgoalconfigvelocities"}};
    if( find(names.begin(),names.end(),name) != names.end() ) {
        __processingtag = name;
        return PE_Support;
    }
    return PE_Pass;
}

bool PlannerParameters::endElement(const std::string& name)
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
        else if( name == "_vinitialconfigvelocities") {
            _vInitialConfigVelocities = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        }
        else if( name == "_vgoalconfig") {
            vgoalconfig = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        }
        else if( name == "_vgoalconfigvelocities") {
            _vGoalConfigVelocities = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
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
        else if( name == "_nmaxplanningtime") {
            _ss >> _nMaxPlanningTime;
        }
        else if( name == "_fsteplength") {
            _ss >> _fStepLength;
        }
        else if( name == "_nrandomgeneratorseed") {
            _ss >> _nRandomGeneratorSeed;
        }
        if( name !=__processingtag ) {
            RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%name%__processingtag));
        }
        __processingtag = "";
        return false;
    }

    return false;
}

void PlannerParameters::characters(const std::string& ch)
{
    if( !!__pcurreader ) {
        __pcurreader->characters(ch);
    }
    else {
        _ss.clear();
        _ss << ch;
    }
}

std::ostream& operator<<(std::ostream& O, const PlannerParameters& v)
{
    O << "<" << v.GetXMLId() << ">" << endl;
    v.serialize(O);
    O << "</" << v.GetXMLId() << ">" << endl;
    return O;
}

int SetActiveDOFValuesParameters(RobotBasePtr probot, const std::vector<dReal>& values, int options)
{
    // should setstatefn check limits?
    probot->SetActiveDOFValues(values, KinBody::CLA_CheckLimits);
    return 0;
}

int SetDOFValuesIndicesParameters(KinBodyPtr pbody, const std::vector<dReal>& values, const std::vector<int>& vindices, int options)
{
    // should setstatefn check limits?
    pbody->SetDOFValues(values, KinBody::CLA_CheckLimits, vindices);
    return 0;
}

int SetDOFVelocitiesIndicesParameters(KinBodyPtr pbody, const std::vector<dReal>& velocities, const std::vector<int>& vindices, int options)
{
    // should setstatefn check limits?
    pbody->SetDOFVelocities(velocities, KinBody::CLA_CheckLimits, vindices);
    return 0;
}

void PlannerParameters::SetRobotActiveJoints(RobotBasePtr robot)
{
    // check if any of the links affected by the dofs beside the base link are static
    FOREACHC(itlink, robot->GetLinks()) {
        if( (*itlink)->IsStatic() ) {
            FOREACHC(itdof, robot->GetActiveDOFIndices()) {
                KinBody::JointPtr pjoint = robot->GetJointFromDOFIndex(*itdof);
                OPENRAVE_ASSERT_FORMAT(!robot->DoesAffect(pjoint->GetJointIndex(),(*itlink)->GetIndex()),"robot %s link %s is static when it is affected by active joint %s", robot->GetName()%(*itlink)->GetName()%pjoint->GetName(), ORE_InvalidState);
            }
        }
    }

    using namespace planningutils;
    _distmetricfn = boost::bind(&SimpleDistanceMetric::Eval,boost::shared_ptr<SimpleDistanceMetric>(new SimpleDistanceMetric(robot)),_1,_2);
    if( robot->GetActiveDOF() == (int)robot->GetActiveDOFIndices().size() ) {
        // only roobt joint indices, so use a more resiliant function
        _getstatefn = boost::bind(&RobotBase::GetDOFValues,robot,_1,robot->GetActiveDOFIndices());
        _setstatevaluesfn = boost::bind(SetDOFValuesIndicesParameters,robot, _1, robot->GetActiveDOFIndices(), _2);
        _diffstatefn = boost::bind(&RobotBase::SubtractDOFValues,robot,_1,_2, robot->GetActiveDOFIndices());
    }
    else {
        _getstatefn = boost::bind(&RobotBase::GetActiveDOFValues,robot,_1);
        _setstatevaluesfn = boost::bind(SetActiveDOFValuesParameters,robot, _1, _2);
        _diffstatefn = boost::bind(&RobotBase::SubtractActiveDOFValues,robot,_1,_2);
    }

    SpaceSamplerBasePtr pconfigsampler = RaveCreateSpaceSampler(robot->GetEnv(),str(boost::format("robotconfiguration %s")%robot->GetName()));
    _listInternalSamplers.clear();
    _listInternalSamplers.push_back(pconfigsampler);

    boost::shared_ptr<SimpleNeighborhoodSampler> defaultsamplefn(new SimpleNeighborhoodSampler(pconfigsampler,_distmetricfn, _diffstatefn));
    _samplefn = boost::bind(&SimpleNeighborhoodSampler::Sample,defaultsamplefn,_1);
    _sampleneighfn = boost::bind(&SimpleNeighborhoodSampler::Sample,defaultsamplefn,_1,_2,_3);

    robot->GetActiveDOFLimits(_vConfigLowerLimit,_vConfigUpperLimit);
    robot->GetActiveDOFVelocityLimits(_vConfigVelocityLimit);
    robot->GetActiveDOFAccelerationLimits(_vConfigAccelerationLimit);
    robot->GetActiveDOFResolutions(_vConfigResolution);
    robot->GetActiveDOFValues(vinitialconfig);
    robot->GetActiveDOFVelocities(_vInitialConfigVelocities); // necessary?
    _configurationspecification = robot->GetActiveConfigurationSpecification();

    _neighstatefn = boost::bind(AddStatesWithLimitCheck, _1, _2, _3, boost::ref(_vConfigLowerLimit), boost::ref(_vConfigUpperLimit)); // probably ok... do we need to clamp limits?

    // have to do this last, disable timed constraints for default
    std::list<KinBodyPtr> listCheckCollisions; listCheckCollisions.push_back(robot);
    boost::shared_ptr<DynamicsCollisionConstraint> pcollision(new DynamicsCollisionConstraint(shared_parameters(), listCheckCollisions,0xffffffff&~CFO_CheckTimeBasedConstraints));
    _checkpathvelocityconstraintsfn = boost::bind(&DynamicsCollisionConstraint::Check,pcollision,_1, _2, _3, _4, _5, _6, _7, _8);

}

void _CallDiffStateFns(const std::vector< std::pair<PlannerParameters::DiffStateFn, int> >& vfunctions, int nDOF, int nMaxDOFForGroup, std::vector<dReal>& v0, const std::vector<dReal>& v1)
{
    if( vfunctions.size() == 1 ) {
        vfunctions.at(0).first(v0,v1);
    }
    else {
        OPENRAVE_ASSERT_OP((int)v0.size(),==,nDOF);
        OPENRAVE_ASSERT_OP((int)v1.size(),==,nDOF);
        std::vector<dReal> vtemp0, vtemp1; vtemp0.reserve(nMaxDOFForGroup); vtemp1.reserve(nMaxDOFForGroup);
        std::vector<dReal>::iterator itsource0 = v0.begin();
        std::vector<dReal>::const_iterator itsource1 = v1.begin();
        FOREACHC(itfn, vfunctions) {
            vtemp0.resize(itfn->second);
            std::copy(itsource0,itsource0+itfn->second,vtemp0.begin());
            vtemp1.resize(itfn->second);
            std::copy(itsource1,itsource1+itfn->second,vtemp1.begin());
            itfn->first(vtemp0, vtemp1);
            // copy result back to itsource0
            std::copy(vtemp0.begin(),vtemp0.end(),itsource0);
            itsource0 += itfn->second;
            itsource1 += itfn->second;
        }
    }
}

/// \brief returns square root of joint distance * weights
/// \param vweights2 squared weights
dReal _EvalJointDOFDistanceMetric(const PlannerParameters::DiffStateFn& difffn, const std::vector<dReal>&c0, const std::vector<dReal>&c1, const std::vector<dReal>& vweights2)
{
    std::vector<dReal> c = c0;
    difffn(c,c1);
    dReal dist = 0;
    for(size_t i=0; i < c.size(); i++) {
        dist += vweights2.at(i)*c.at(i)*c.at(i);
    }
    return RaveSqrt(dist);
}

dReal _CallDistMetricFns(const std::vector< std::pair<PlannerParameters::DistMetricFn, int> >& vfunctions, int nDOF, int nMaxDOFForGroup, const std::vector<dReal>& v0, const std::vector<dReal>& v1)
{
    if( vfunctions.size() == 1 ) {
        return vfunctions.at(0).first(v0, v1);
    }
    else {
        OPENRAVE_ASSERT_OP((int)v0.size(),==,nDOF);
        OPENRAVE_ASSERT_OP((int)v1.size(),==,nDOF);
        std::vector<dReal> vtemp0, vtemp1; vtemp0.reserve(nMaxDOFForGroup); vtemp1.reserve(nMaxDOFForGroup);
        std::vector<dReal>::const_iterator itsource0 = v0.begin(), itsource1 = v1.begin();
        dReal f = 0;
        FOREACHC(itfn, vfunctions) {
            vtemp0.resize(itfn->second);
            std::copy(itsource0,itsource0+itfn->second,vtemp0.begin());
            vtemp1.resize(itfn->second);
            std::copy(itsource1,itsource1+itfn->second,vtemp1.begin());
            f += itfn->first(vtemp0, vtemp1);
            itsource0 += itfn->second;
            itsource1 += itfn->second;
        }
        return f;
    }
}

bool _CallSampleFns(const std::vector< std::pair<PlannerParameters::SampleFn, int> >& vfunctions, int nDOF, int nMaxDOFForGroup, std::vector<dReal>& v)
{
    if( vfunctions.size() == 1 ) {
        return vfunctions.at(0).first(v);
    }
    else {
        std::vector<dReal> vtemp; vtemp.reserve(nMaxDOFForGroup);
        v.resize(nDOF);
        std::vector<dReal>::iterator itdest = v.begin();
        FOREACHC(itfn, vfunctions) {
            if( !itfn->first(vtemp) ) {
                return false;
            }
            std::copy(vtemp.begin(),vtemp.end(),itdest);
            itdest += itfn->second;
        }
        return true;
    }
}

bool _CallSampleNeighFns(const std::vector< std::pair<PlannerParameters::SampleNeighFn, int> >& vfunctions, const std::vector< std::pair<PlannerParameters::DistMetricFn, int> >& vdistfunctions, int nDOF, int nMaxDOFForGroup, std::vector<dReal>& v, const std::vector<dReal>& vCurSample, dReal fRadius)
{
    if( vfunctions.size() == 1 ) {
        return vfunctions.at(0).first(v,vCurSample,fRadius);
    }
    else {
        OPENRAVE_ASSERT_OP((int)vCurSample.size(),==,nDOF);
        std::vector<dReal> vtemp0, vtemp1; vtemp0.reserve(nMaxDOFForGroup); vtemp1.reserve(nMaxDOFForGroup);
        std::vector<dReal>::const_iterator itsample = vCurSample.begin();
        v.resize(nDOF);
        std::vector<dReal>::iterator itdest = v.begin();
        int ifn = 0;
        FOREACHC(itfn, vfunctions) {
            vtemp1.resize(itfn->second);
            if( fRadius <= 0 ) {
                std::copy(itsample,itsample+itfn->second,itdest);
            }
            else {
                std::copy(itsample,itsample+itfn->second,vtemp1.begin());
                if( !itfn->first(vtemp0,vtemp1,fRadius) ) {
                    return false;
                }
                fRadius -= vdistfunctions[ifn].first(vtemp0,vtemp1);
                std::copy(vtemp0.begin(),vtemp0.end(),itdest);
            }
            itdest += itfn->second;
            itsample += itfn->second;
            ++ifn;
        }
        return true;
    }
}

int CallSetStateValuesFns(const std::vector< std::pair<PlannerParameters::SetStateValuesFn, int> >& vfunctions, int nDOF, int nMaxDOFForGroup, const std::vector<dReal>& v, int options)
{
    if( vfunctions.size() == 1 ) {
        return vfunctions.at(0).first(v, options);
    }
    else {
        std::vector<dReal> vtemp; vtemp.reserve(nMaxDOFForGroup);
        OPENRAVE_ASSERT_OP((int)v.size(),==,nDOF);
        std::vector<dReal>::const_iterator itsrc = v.begin();
        FOREACHC(itfn, vfunctions) {
            vtemp.resize(itfn->second);
            std::copy(itsrc,itsrc+itfn->second,vtemp.begin());
            int ret = itfn->first(vtemp, options);
            if( ret != 0 ) {
                return ret;
            }
            itsrc += itfn->second;
        }
    }
    return 0;
}

void CallGetStateFns(const std::vector< std::pair<PlannerParameters::GetStateFn, int> >& vfunctions, int nDOF, int nMaxDOFForGroup, std::vector<dReal>& v)
{
    if( vfunctions.size() == 1 ) {
        vfunctions.at(0).first(v);
    }
    else {
        std::vector<dReal> vtemp; vtemp.reserve(nMaxDOFForGroup);
        v.resize(nDOF);
        std::vector<dReal>::iterator itdest = v.begin();
        FOREACHC(itfn, vfunctions) {
            itfn->first(vtemp);
            std::copy(vtemp.begin(),vtemp.end(),itdest);
            itdest += itfn->second;
        }
    }
}

int _CallNeighStateFns(const std::vector< std::pair<PlannerParameters::NeighStateFn, int> >& vfunctions, int nDOF, int nMaxDOFForGroup, std::vector<dReal>& v, const std::vector<dReal>& vdelta, int fromgoal)
{
    if( vfunctions.size() == 1 ) {
        return vfunctions.at(0).first(v,vdelta,fromgoal);
    }
    else {
        OPENRAVE_ASSERT_OP((int)vdelta.size(),==,nDOF);
        OPENRAVE_ASSERT_OP((int)v.size(),==,nDOF);
        std::vector<dReal> vtemp0, vtemp1; vtemp0.reserve(nMaxDOFForGroup); vtemp1.reserve(nMaxDOFForGroup);
        std::vector<dReal>::const_iterator itdelta = vdelta.begin();
        std::vector<dReal>::iterator itdest = v.begin();
        int ret = NSS_Failed;
        FOREACHC(itfn, vfunctions) {
            vtemp0.resize(itfn->second);
            std::copy(itdest,itdest+itfn->second,vtemp0.begin());
            vtemp1.resize(itfn->second);
            std::copy(itdelta,itdelta+itfn->second,vtemp1.begin());
            int status = itfn->first(vtemp0,vtemp1,fromgoal);
            if( status == NSS_Failed ) {
                return NSS_Failed;
            }
            ret |= status;
            std::copy(vtemp0.begin(),vtemp0.end(),itdest);
            itdest += itfn->second;
            itdelta += itfn->second;
        }
        return ret;
    }
}

void PlannerParameters::SetConfigurationSpecification(EnvironmentBasePtr penv, const ConfigurationSpecification& spec)
{
    using namespace planningutils;
    spec.Validate();
    std::vector< std::pair<DiffStateFn, int> > diffstatefns(spec._vgroups.size());
    std::vector< std::pair<DistMetricFn, int> > distmetricfns(spec._vgroups.size());
    std::vector< std::pair<SampleFn, int> > samplefns(spec._vgroups.size());
    std::vector< std::pair<SampleNeighFn, int> > sampleneighfns(spec._vgroups.size());
    std::vector< std::pair<SetStateValuesFn, int> > setstatevaluesfns(spec._vgroups.size());
    std::vector< std::pair<GetStateFn, int> > getstatefns(spec._vgroups.size());
    std::vector< std::pair<NeighStateFn, int> > neighstatefns(spec._vgroups.size());
    std::vector<dReal> vConfigLowerLimit(spec.GetDOF()), vConfigUpperLimit(spec.GetDOF()), vConfigVelocityLimit(spec.GetDOF()), vConfigAccelerationLimit(spec.GetDOF()), vConfigResolution(spec.GetDOF()), v0, v1;
    std::list<KinBodyPtr> listCheckCollisions;
    string bodyname;
    stringstream ss, ssout;
    // order the groups depending on offset
    int nMaxDOFForGroup = 0;
    std::vector< std::pair<int, int> > vgroupoffsets(spec._vgroups.size());
    for(size_t igroup = 0; igroup < spec._vgroups.size(); ++igroup) {
        vgroupoffsets[igroup].first = spec._vgroups[igroup].offset;
        vgroupoffsets[igroup].second = igroup;
        nMaxDOFForGroup = max(nMaxDOFForGroup,spec._vgroups[igroup].dof);
    }

    _listInternalSamplers.clear();

    std::sort(vgroupoffsets.begin(),vgroupoffsets.end());
    for(size_t igroup = 0; igroup < spec._vgroups.size(); ++igroup) {
        const ConfigurationSpecification::Group& g = spec._vgroups[igroup];
        int isavegroup = vgroupoffsets[igroup].second;
        if( g.name.size() >= 12 && g.name.substr(0,12) == "joint_values" ) {
            ss.clear(); ss.str(g.name.substr(12));
            ss >> bodyname;
            BOOST_ASSERT(!!ss);
            KinBodyPtr pbody = penv->GetKinBody(bodyname);
            OPENRAVE_ASSERT_FORMAT(!!pbody,"body %s not found",bodyname,ORE_InvalidArguments);
            std::vector<int> dofindices((istream_iterator<int>(ss)), istream_iterator<int>());
            if( dofindices.size() == 0 ) {
                OPENRAVE_ASSERT_OP((int)dofindices.size(),==,pbody->GetDOF());
            }
            std::vector<dReal> vweights2;
            pbody->GetDOFWeights(vweights2, dofindices);
            FOREACH(itf,vweights2) {
                *itf *= *itf;
            }
            diffstatefns[isavegroup].first = boost::bind(&KinBody::SubtractDOFValues, pbody, _1, _2, dofindices);
            diffstatefns[isavegroup].second = g.dof;
            distmetricfns[isavegroup].first = boost::bind(_EvalJointDOFDistanceMetric, diffstatefns[isavegroup].first, _1, _2, vweights2);
            distmetricfns[isavegroup].second = g.dof;

            SpaceSamplerBasePtr pconfigsampler = RaveCreateSpaceSampler(penv,str(boost::format("bodyconfiguration %s")%pbody->GetName()));
            _listInternalSamplers.push_back(pconfigsampler);
            {
                ss.clear(); ss.str(""); ss << "SetDOFs ";
                FOREACHC(itindex,dofindices) {
                    ss << *itindex << " ";
                }
                if( !pconfigsampler->SendCommand(ssout,ss) ) {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("failed to set body %s configuration to %s"),pbody->GetName()%ss.str(), ORE_Assert);
                }
            }
            boost::shared_ptr<SimpleNeighborhoodSampler> defaultsamplefn(new SimpleNeighborhoodSampler(pconfigsampler,distmetricfns[isavegroup].first, diffstatefns[isavegroup].first));
            samplefns[isavegroup].first = boost::bind(&SimpleNeighborhoodSampler::Sample,defaultsamplefn,_1);
            samplefns[isavegroup].second = g.dof;
            sampleneighfns[isavegroup].first = boost::bind(&SimpleNeighborhoodSampler::Sample,defaultsamplefn,_1,_2,_3);
            sampleneighfns[isavegroup].second = g.dof;
            setstatevaluesfns[isavegroup].first = boost::bind(SetDOFValuesIndicesParameters, pbody, _1, dofindices, _2);
            setstatevaluesfns[isavegroup].second = g.dof;
            getstatefns[isavegroup].first = boost::bind(&KinBody::GetDOFValues, pbody, _1, dofindices);
            getstatefns[isavegroup].second = g.dof;
            neighstatefns[isavegroup].second = g.dof;
            pbody->GetDOFLimits(v0,v1,dofindices);
            neighstatefns[isavegroup].first = boost::bind(AddStatesWithLimitCheck, _1, _2, _3, v0, v1);
            std::copy(v0.begin(),v0.end(), vConfigLowerLimit.begin()+g.offset);
            std::copy(v1.begin(),v1.end(), vConfigUpperLimit.begin()+g.offset);
            pbody->GetDOFVelocityLimits(v0,dofindices);
            std::copy(v0.begin(),v0.end(), vConfigVelocityLimit.begin()+g.offset);
            pbody->GetDOFAccelerationLimits(v0,dofindices);
            std::copy(v0.begin(),v0.end(), vConfigAccelerationLimit.begin()+g.offset);
            pbody->GetDOFResolutions(v0,dofindices);
            std::copy(v0.begin(),v0.end(),vConfigResolution.begin()+g.offset);
            if( find(listCheckCollisions.begin(),listCheckCollisions.end(),pbody) == listCheckCollisions.end() ) {
                listCheckCollisions.push_back(pbody);
            }
        }
//        else if( g.name.size() >= 16 && g.name.substr(0,16) == "joint_velocities" ) {
//        }
//        else if( g.name.size() >= 16 && g.name.substr(0,16) == "affine_transform" ) {
//        }
//        else if( g.name.size() >= 17 && g.name.substr(0,17) == "affine_velocities" ) {
//        }
//        else if( g.name.size() >= 4 && g.name.substr(0,4) == "grab" ) {
//        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT(_("group %s not supported for for planner parameters configuration"),g.name,ORE_InvalidArguments);
        }
    }
    _diffstatefn = boost::bind(_CallDiffStateFns,diffstatefns, spec.GetDOF(), nMaxDOFForGroup, _1, _2);
    _distmetricfn = boost::bind(_CallDistMetricFns,distmetricfns, spec.GetDOF(), nMaxDOFForGroup, _1, _2);
    _samplefn = boost::bind(_CallSampleFns,samplefns, spec.GetDOF(), nMaxDOFForGroup, _1);
    _sampleneighfn = boost::bind(_CallSampleNeighFns,sampleneighfns, distmetricfns, spec.GetDOF(), nMaxDOFForGroup, _1, _2, _3);
    _setstatevaluesfn = boost::bind(CallSetStateValuesFns,setstatevaluesfns, spec.GetDOF(), nMaxDOFForGroup, _1, _2);
    _getstatefn = boost::bind(CallGetStateFns,getstatefns, spec.GetDOF(), nMaxDOFForGroup, _1);
    _neighstatefn = boost::bind(_CallNeighStateFns,neighstatefns, spec.GetDOF(), nMaxDOFForGroup, _1,_2,_3);
    _vConfigLowerLimit.swap(vConfigLowerLimit);
    _vConfigUpperLimit.swap(vConfigUpperLimit);
    _vConfigVelocityLimit.swap(vConfigVelocityLimit);
    _vConfigAccelerationLimit.swap(vConfigAccelerationLimit);
    _vConfigResolution.swap(vConfigResolution);
    _configurationspecification = spec;
    _getstatefn(vinitialconfig);
    // have to do this last, disable timed constraints for default
    boost::shared_ptr<DynamicsCollisionConstraint> pcollision(new DynamicsCollisionConstraint(shared_parameters(), listCheckCollisions,0xffffffff&~CFO_CheckTimeBasedConstraints));
    _checkpathvelocityconstraintsfn = boost::bind(&DynamicsCollisionConstraint::Check,pcollision,_1, _2, _3, _4, _5, _6, _7, _8);
}

void PlannerParameters::Validate() const
{
    OPENRAVE_ASSERT_OP(_configurationspecification.GetDOF(),==,GetDOF());
    OPENRAVE_ASSERT_OP(vinitialconfig.size()%GetDOF(),==,0);
    OPENRAVE_ASSERT_OP(_vInitialConfigVelocities.size()%GetDOF(),==,0);
    OPENRAVE_ASSERT_OP(_vGoalConfigVelocities.size()%GetDOF(),==,0);
    OPENRAVE_ASSERT_OP(vgoalconfig.size()%GetDOF(),==,0);
    OPENRAVE_ASSERT_OP(_vConfigLowerLimit.size(),==,(size_t)GetDOF());
    OPENRAVE_ASSERT_OP(_vConfigUpperLimit.size(),==,(size_t)GetDOF());
    if( _vConfigVelocityLimit.size() > 0 ) {
        OPENRAVE_ASSERT_OP(_vConfigVelocityLimit.size(),==,(size_t)GetDOF());
    }
    if( _vConfigAccelerationLimit.size() > 0 ) {
        OPENRAVE_ASSERT_OP(_vConfigAccelerationLimit.size(),==,(size_t)GetDOF());
    }
    OPENRAVE_ASSERT_OP(_vConfigResolution.size(),==,(size_t)GetDOF());
    OPENRAVE_ASSERT_OP(_fStepLength,>=,0); // == 0 is valid for auto-steps
    OPENRAVE_ASSERT_OP(_nMaxIterations,>=,0); // == 0 is valid for auto-iterations

    // check all stateless functions, which means ie anything but configuration samplers
    vector<dReal> vstate;
    if( !!_getstatefn ) {
        _getstatefn(vstate);
        OPENRAVE_ASSERT_OP(vstate.size(),==,(size_t)GetDOF());
    }
    if( !!_setstatevaluesfn ) {
        // need to save/restore state before calling this function?
        //_setstatefn();
    }
    if( !!_costfn ) {
        _costfn(vstate);
    }
    if( !!_goalfn ) {
        _goalfn(vstate);
    }
    if( !!_distmetricfn ) {
        dReal dist = _distmetricfn(vstate,vstate);
        OPENRAVE_ASSERT_OP(dist,<=,10*g_fEpsilon);
    }
    if( !!_checkpathvelocityconstraintsfn ) {
        _checkpathvelocityconstraintsfn(vstate,vstate,std::vector<dReal>(), std::vector<dReal>(), 0, IT_OpenStart,0,ConstraintFilterReturnPtr());
    }
    if( !!_neighstatefn && vstate.size() > 0 ) {
        vector<dReal> vstate2 = vstate;
        vector<dReal> vzeros(vstate.size());
        int neighstatus = _neighstatefn(vstate2,vzeros,NSO_OnlyHardConstraints);
        OPENRAVE_ASSERT_OP(neighstatus,&,NSS_Reached); // LSB indicates if _neighstatefn call is successful
        dReal dist = _distmetricfn(vstate,vstate2);
        if( IS_DEBUGLEVEL(Level_Debug) ) {
            if( dist > 1000*g_fEpsilon ) {
                std::stringstream ss; ss << "vstate=";
                for(size_t i = 0; i < vstate.size(); ++i) {
                    ss << vstate[i] << ", ";
                }
                ss << "; vstate2=";
                for(size_t i = 0; i < vstate2.size(); ++i) {
                    ss << vstate2[i] << ", ";
                }
                RAVELOG_DEBUG_FORMAT("unequal states: %s",ss.str());
            }
        }

        OPENRAVE_ASSERT_OP(dist,<=,1000*g_fEpsilon);
    }
    if( !!_diffstatefn && vstate.size() > 0 ) {
        vector<dReal> vstate2=vstate;
        _diffstatefn(vstate2,vstate);
        OPENRAVE_ASSERT_OP(vstate2.size(),==,(size_t)GetDOF());
    }
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
    localparams->Validate();
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
        __cachePostProcessPlanner.reset();
        return PlannerStatus(PS_HasSolution);
    }
    if( !__cachePostProcessPlanner || __cachePostProcessPlanner->GetXMLId() != GetParameters()->_sPostProcessingPlanner ) {
        __cachePostProcessPlanner = RaveCreatePlanner(GetEnv(), GetParameters()->_sPostProcessingPlanner);
        if( !__cachePostProcessPlanner ) {
            __cachePostProcessPlanner = RaveCreatePlanner(GetEnv(), s_linearsmoother);
            if( !__cachePostProcessPlanner ) {
                return PlannerStatus(PS_Failed);
            }
        }
    }

    // transfer the callbacks?
    list<UserDataPtr> listhandles;
    FOREACHC(it,__listRegisteredCallbacks) {
        CustomPlannerCallbackDataPtr pitdata = boost::dynamic_pointer_cast<CustomPlannerCallbackData>(it->lock());
        if( !!pitdata) {
            listhandles.push_back(__cachePostProcessPlanner->RegisterPlanCallback(pitdata->_callbackfn));
        }
    }

    PlannerParametersPtr params(new PlannerParameters());
    params->copy(GetParameters());
    params->_sExtraParameters += GetParameters()->_sPostProcessingParameters;
    params->_sPostProcessingPlanner = "";
    params->_sPostProcessingParameters = "";
    params->_nMaxIterations = 0; // have to reset since path optimizers also use it and new parameters could be in extra parameters
    //params->_nMaxPlanningTime = 0; // have to reset since path optimizers also use it and new parameters could be in extra parameters??
    if( __cachePostProcessPlanner->InitPlan(probot, params) ) {
        return __cachePostProcessPlanner->PlanPath(ptraj);
    }

    // do not fall back to a default linear smoother like in the past! that makes behavior unpredictable
    return PlannerStatus(PS_Failed);
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
