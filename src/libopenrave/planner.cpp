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

namespace OpenRAVE {

static std::string s_linearsmoother = "linearsmoother"; //"shortcut_linear";

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
        LocalXML::ParseXMLData(PlannerBase::PlannerParametersPtr(&pp,utils::null_deleter()), pbuf.c_str(), ppsize);
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

PlannerBase::PlannerParameters::StateSaver::StateSaver(PlannerParametersPtr params) : _params(params)
{
    BOOST_ASSERT(!!_params->_setstatefn);
    _params->_getstatefn(_values);
    OPENRAVE_ASSERT_OP((int)_values.size(),==,_params->GetDOF());
}

PlannerBase::PlannerParameters::StateSaver::~StateSaver()
{
    _Restore();
}

void PlannerBase::PlannerParameters::StateSaver::Restore()
{
    _Restore();
}

void PlannerBase::PlannerParameters::StateSaver::_Restore()
{
    _params->_setstatefn(_values);
}

PlannerBase::PlannerParameters::PlannerParameters() : XMLReadable("plannerparameters"), _fStepLength(0.04f), _nMaxIterations(0), _sPostProcessingPlanner(s_linearsmoother)
{
    _diffstatefn = subtractstates;
    _neighstatefn = addstates;
    //_sPostProcessingParameters ="<_nmaxiterations>100</_nmaxiterations><_postprocessing planner=\"lineartrajectoryretimer\"></_postprocessing>";
    _sPostProcessingParameters ="<_nmaxiterations>20</_nmaxiterations><_postprocessing planner=\"parabolicsmoother\"><_nmaxiterations>100</_nmaxiterations></_postprocessing>";
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

bool PlannerBase::PlannerParameters::serialize(std::ostream& O, int options) const
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
    if( !(options & 1) ) {
        O << _sExtraParameters << endl;
    }
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
    SpaceSamplerBasePtr pconfigsampler = RaveCreateSpaceSampler(robot->GetEnv(),str(boost::format("robotconfiguration %s")%robot->GetName()));
    boost::shared_ptr<SimpleNeighborhoodSampler> defaultsamplefn(new SimpleNeighborhoodSampler(pconfigsampler,_distmetricfn));
    _samplefn = boost::bind(&SimpleNeighborhoodSampler::Sample,defaultsamplefn,_1);
    _sampleneighfn = boost::bind(&SimpleNeighborhoodSampler::Sample,defaultsamplefn,_1,_2,_3);
    _setstatefn = boost::bind(&RobotBase::SetActiveDOFValues,robot,_1,false);
    _getstatefn = boost::bind(&RobotBase::GetActiveDOFValues,robot,_1);
    _diffstatefn = boost::bind(&RobotBase::SubtractActiveDOFValues,robot,_1,_2);
    _neighstatefn = addstates; // probably ok...

    std::list<KinBodyPtr> listCheckCollisions; listCheckCollisions.push_back(robot);
    boost::shared_ptr<LineCollisionConstraint> pcollision(new LineCollisionConstraint(listCheckCollisions,true));
    _checkpathconstraintsfn = boost::bind(&LineCollisionConstraint::Check,pcollision,PlannerParametersWeakPtr(shared_parameters()), _1, _2, _3, _4);

    robot->GetActiveDOFLimits(_vConfigLowerLimit,_vConfigUpperLimit);
    robot->GetActiveDOFVelocityLimits(_vConfigVelocityLimit);
    robot->GetActiveDOFAccelerationLimits(_vConfigAccelerationLimit);
    robot->GetActiveDOFResolutions(_vConfigResolution);
    robot->GetActiveDOFValues(vinitialconfig);
    _configurationspecification = robot->GetActiveConfigurationSpecification();
}

void _CallDiffStateFns(const std::vector< std::pair<PlannerBase::PlannerParameters::DiffStateFn, int> >& vfunctions, int nDOF, int nMaxDOFForGroup, std::vector<dReal>& v0, const std::vector<dReal>& v1)
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
dReal _EvalJointDOFDistanceMetric(const PlannerBase::PlannerParameters::DiffStateFn& difffn, const std::vector<dReal>&c0, const std::vector<dReal>&c1, const std::vector<dReal>& vweights2)
{
    std::vector<dReal> c = c0;
    difffn(c,c1);
    dReal dist = 0;
    for(size_t i=0; i < c.size(); i++) {
        dist += vweights2.at(i)*c.at(i)*c.at(i);
    }
    return RaveSqrt(dist);
}

dReal _CallDistMetricFns(const std::vector< std::pair<PlannerBase::PlannerParameters::DistMetricFn, int> >& vfunctions, int nDOF, int nMaxDOFForGroup, const std::vector<dReal>& v0, const std::vector<dReal>& v1)
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

bool _CallSampleFns(const std::vector< std::pair<PlannerBase::PlannerParameters::SampleFn, int> >& vfunctions, int nDOF, int nMaxDOFForGroup, std::vector<dReal>& v)
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

bool _CallSampleNeighFns(const std::vector< std::pair<PlannerBase::PlannerParameters::SampleNeighFn, int> >& vfunctions, const std::vector< std::pair<PlannerBase::PlannerParameters::DistMetricFn, int> >& vdistfunctions, int nDOF, int nMaxDOFForGroup, std::vector<dReal>& v, const std::vector<dReal>& vCurSample, dReal fRadius)
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

void CallSetStateFns(const std::vector< std::pair<PlannerBase::PlannerParameters::SetStateFn, int> >& vfunctions, int nDOF, int nMaxDOFForGroup, const std::vector<dReal>& v)
{
    if( vfunctions.size() == 1 ) {
        return vfunctions.at(0).first(v);
    }
    else {
        std::vector<dReal> vtemp; vtemp.reserve(nMaxDOFForGroup);
        OPENRAVE_ASSERT_OP((int)v.size(),==,nDOF);
        std::vector<dReal>::const_iterator itsrc = v.begin();
        FOREACHC(itfn, vfunctions) {
            vtemp.resize(itfn->second);
            std::copy(itsrc,itsrc+itfn->second,vtemp.begin());
            itfn->first(vtemp);
            itsrc += itfn->second;
        }
    }
}

void CallGetStateFns(const std::vector< std::pair<PlannerBase::PlannerParameters::GetStateFn, int> >& vfunctions, int nDOF, int nMaxDOFForGroup, std::vector<dReal>& v)
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

bool _CallNeighStateFns(const std::vector< std::pair<PlannerBase::PlannerParameters::NeighStateFn, int> >& vfunctions, int nDOF, int nMaxDOFForGroup, std::vector<dReal>& v, const std::vector<dReal>& vdelta, int fromgoal)
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
        FOREACHC(itfn, vfunctions) {
            vtemp0.resize(itfn->second);
            std::copy(itdest,itdest+itfn->second,vtemp0.begin());
            vtemp1.resize(itfn->second);
            std::copy(itdelta,itdelta+itfn->second,vtemp1.begin());
            if( !itfn->first(vtemp0,vtemp1,fromgoal) ) {
                return false;
            }
            std::copy(vtemp0.begin(),vtemp0.end(),itdest);
            itdest += itfn->second;
            itdelta += itfn->second;
        }
        return true;
    }
}

void PlannerBase::PlannerParameters::SetConfigurationSpecification(EnvironmentBasePtr penv, const ConfigurationSpecification& spec)
{
    using namespace planningutils;
    spec.Validate();
    std::vector< std::pair<DiffStateFn, int> > diffstatefns(spec._vgroups.size());
    std::vector< std::pair<DistMetricFn, int> > distmetricfns(spec._vgroups.size());
    std::vector< std::pair<SampleFn, int> > samplefns(spec._vgroups.size());
    std::vector< std::pair<SampleNeighFn, int> > sampleneighfns(spec._vgroups.size());
    std::vector< std::pair<SetStateFn, int> > setstatefns(spec._vgroups.size());
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
            {
                ss.clear(); ss.str(""); ss << "SetDOFs ";
                FOREACHC(itindex,dofindices) {
                    ss << *itindex << " ";
                }
                if( !pconfigsampler->SendCommand(ssout,ss) ) {
                    throw OPENRAVE_EXCEPTION_FORMAT("failed to set body %s configuration to %s",pbody->GetName()%ss.str(), ORE_Assert);
                }
            }
            boost::shared_ptr<SimpleNeighborhoodSampler> defaultsamplefn(new SimpleNeighborhoodSampler(pconfigsampler,distmetricfns[isavegroup].first));
            samplefns[isavegroup].first = boost::bind(&SimpleNeighborhoodSampler::Sample,defaultsamplefn,_1);
            samplefns[isavegroup].second = g.dof;
            sampleneighfns[isavegroup].first = boost::bind(&SimpleNeighborhoodSampler::Sample,defaultsamplefn,_1,_2,_3);
            sampleneighfns[isavegroup].second = g.dof;
            void (KinBody::*setdofvaluesptr)(const std::vector<dReal>&, uint32_t, const std::vector<int>&) = &KinBody::SetDOFValues;
            setstatefns[isavegroup].first = boost::bind(setdofvaluesptr, pbody, _1, KinBody::CLA_CheckLimits, dofindices);
            setstatefns[isavegroup].second = g.dof;
            getstatefns[isavegroup].first = boost::bind(&KinBody::GetDOFValues, pbody, _1, dofindices);
            getstatefns[isavegroup].second = g.dof;
            neighstatefns[isavegroup].first = addstates;
            neighstatefns[isavegroup].second = g.dof;
            pbody->GetDOFLimits(v0,v1,dofindices);
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
            throw OPENRAVE_EXCEPTION_FORMAT("group %s not supported for for planner parameters configuration",g.name,ORE_InvalidArguments);
        }
    }
    _diffstatefn = boost::bind(_CallDiffStateFns,diffstatefns, spec.GetDOF(), nMaxDOFForGroup, _1, _2);
    _distmetricfn = boost::bind(_CallDistMetricFns,distmetricfns, spec.GetDOF(), nMaxDOFForGroup, _1, _2);
    _samplefn = boost::bind(_CallSampleFns,samplefns, spec.GetDOF(), nMaxDOFForGroup, _1);
    _sampleneighfn = boost::bind(_CallSampleNeighFns,sampleneighfns, distmetricfns, spec.GetDOF(), nMaxDOFForGroup, _1, _2, _3);
    _setstatefn = boost::bind(CallSetStateFns,setstatefns, spec.GetDOF(), nMaxDOFForGroup, _1);
    _getstatefn = boost::bind(CallGetStateFns,getstatefns, spec.GetDOF(), nMaxDOFForGroup, _1);
    _neighstatefn = boost::bind(_CallNeighStateFns,neighstatefns, spec.GetDOF(), nMaxDOFForGroup, _1,_2,_3);
    boost::shared_ptr<LineCollisionConstraint> pcollision(new LineCollisionConstraint(listCheckCollisions,true));
    _checkpathconstraintsfn = boost::bind(&LineCollisionConstraint::Check,pcollision,PlannerParametersWeakPtr(shared_parameters()), _1, _2, _3, _4);
    _vConfigLowerLimit.swap(vConfigLowerLimit);
    _vConfigUpperLimit.swap(vConfigUpperLimit);
    _vConfigVelocityLimit.swap(vConfigVelocityLimit);
    _vConfigAccelerationLimit.swap(vConfigAccelerationLimit);
    _vConfigResolution.swap(vConfigResolution);
    _configurationspecification = spec;
    _getstatefn(vinitialconfig);
}

void PlannerBase::PlannerParameters::Validate() const
{
    OPENRAVE_ASSERT_OP(_configurationspecification.GetDOF(),==,GetDOF());
    OPENRAVE_ASSERT_OP(vinitialconfig.size()%GetDOF(),==,0);
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
    if( !!_setstatefn ) {
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
    if( !!_checkpathconstraintsfn ) {
        _checkpathconstraintsfn(vstate,vstate,IT_OpenStart,ConfigurationListPtr());
    }
    if( !!_neighstatefn && vstate.size() > 0 ) {
        vector<dReal> vstate2 = vstate;
        vector<dReal> vzeros(vstate.size());
        _neighstatefn(vstate2,vzeros,0);
        dReal dist = _distmetricfn(vstate,vstate2);
        OPENRAVE_ASSERT_OP(dist,<=,10*g_fEpsilon);
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
        return PS_HasSolution;
    }
    PlannerBasePtr planner = RaveCreatePlanner(GetEnv(), GetParameters()->_sPostProcessingPlanner);
    if( !planner ) {
        planner = RaveCreatePlanner(GetEnv(), s_linearsmoother);
        if( !planner ) {
            return PS_Failed;
        }
    }

    // transfer the callbacks?
    list<UserDataPtr> listhandles;
    FOREACHC(it,__listRegisteredCallbacks) {
        CustomPlannerCallbackDataPtr pitdata = boost::dynamic_pointer_cast<CustomPlannerCallbackData>(it->lock());
        if( !!pitdata) {
            listhandles.push_back(planner->RegisterPlanCallback(pitdata->_callbackfn));
        }
    }

    PlannerParametersPtr params(new PlannerParameters());
    params->copy(GetParameters());
    params->_sExtraParameters += GetParameters()->_sPostProcessingParameters;
    params->_sPostProcessingPlanner = "";
    params->_sPostProcessingParameters = "";
    params->_nMaxIterations = 0; // have to reset since path optimizers also use it and new parameters could be in extra parameters
    if( planner->InitPlan(probot, params) ) {
        return planner->PlanPath(ptraj);
    }

    // do not fall back to a default linear smoother like in the past! that makes behavior unpredictable
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
