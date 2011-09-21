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

#define LIBXML_SAX1_ENABLED
#include <libxml/globals.h>
#include <libxml/xmlerror.h>
#include <libxml/parser.h>
#include <libxml/parserInternals.h> // only for xmlNewInputFromFile()
#include <libxml/tree.h>

#include <libxml/debugXML.h>
#include <libxml/xmlmemory.h>

namespace OpenRAVE {

namespace LocalXML {

void RaveXMLErrorFunc(void *ctx, const char *msg, ...)
{
    va_list args;

    va_start(args, msg);
    RAVELOG_ERROR("XML Parse error: ");
    vprintf(msg,args);
    va_end(args);
}

struct XMLREADERDATA
{
    XMLREADERDATA(BaseXMLReaderPtr preader, xmlParserCtxtPtr ctxt) : _preader(preader), _ctxt(ctxt) {
    }
    BaseXMLReaderPtr _preader, _pdummy;
    xmlParserCtxtPtr _ctxt;
};

void DefaultStartElementSAXFunc(void *ctx, const xmlChar *name, const xmlChar **atts)
{
    AttributesList listatts;
    if( atts != NULL ) {
        for (int i = 0; (atts[i] != NULL); i+=2) {
            listatts.push_back(make_pair(string((const char*)atts[i]),string((const char*)atts[i+1])));
            std::transform(listatts.back().first.begin(), listatts.back().first.end(), listatts.back().first.begin(), ::tolower);
        }
    }

    XMLREADERDATA* pdata = (XMLREADERDATA*)ctx;
    string s = (const char*)name;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    if( !!pdata->_pdummy ) {
        RAVELOG_VERBOSE(str(boost::format("unknown field %s\n")%s));
        pdata->_pdummy->startElement(s,listatts);
    }
    else {
        if( ((XMLREADERDATA*)ctx)->_preader->startElement(s, listatts) != BaseXMLReader::PE_Support ) {
            // not handling, so create a temporary class to handle it
            pdata->_pdummy.reset(new DummyXMLReader(s,"(libxml)"));
        }
    }
}

void DefaultEndElementSAXFunc(void *ctx, const xmlChar *name)
{
    XMLREADERDATA* pdata = (XMLREADERDATA*)ctx;
    string s = (const char*)name;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    if( !!pdata->_pdummy ) {
        if( pdata->_pdummy->endElement(s) ) {
            pdata->_pdummy.reset();
        }
    }
    else {
        if( pdata->_preader->endElement(s) ) {
            //RAVEPRINT(L"%s size read %d\n", name, data->_ctxt->input->consumed);
            xmlStopParser(pdata->_ctxt);
        }
    }
}

void DefaultCharactersSAXFunc(void *ctx, const xmlChar *ch, int len)
{
    XMLREADERDATA* pdata = (XMLREADERDATA*)ctx;
    if( !!pdata->_pdummy ) {
        pdata->_pdummy->characters(string((const char*)ch, len));
    }
    else {
        pdata->_preader->characters(string((const char*)ch, len));
    }
}

bool xmlDetectSAX2(xmlParserCtxtPtr ctxt)
{
    if (ctxt == NULL) {
        return false;
    }
#ifdef LIBXML_SAX1_ENABLED
    if ((ctxt->sax) &&  (ctxt->sax->initialized == XML_SAX2_MAGIC) && ((ctxt->sax->startElementNs != NULL) || (ctxt->sax->endElementNs != NULL))) {
        ctxt->sax2 = 1;
    }
#else
    ctxt->sax2 = 1;
#endif

    ctxt->str_xml = xmlDictLookup(ctxt->dict, BAD_CAST "xml", 3);
    ctxt->str_xmlns = xmlDictLookup(ctxt->dict, BAD_CAST "xmlns", 5);
    ctxt->str_xml_ns = xmlDictLookup(ctxt->dict, XML_XML_NAMESPACE, 36);
    if ((ctxt->str_xml==NULL) || (ctxt->str_xmlns==NULL) || (ctxt->str_xml_ns == NULL)) {
        return false;
    }
    return true;
}

bool ParseXMLData(BaseXMLReaderPtr preader, const char* buffer, int size)
{
    static xmlSAXHandler s_DefaultSAXHandler = { 0};
    if( size <= 0 ) {
        size = strlen(buffer);
    }
    if( !s_DefaultSAXHandler.initialized ) {
        // first time, so init
        s_DefaultSAXHandler.startElement = DefaultStartElementSAXFunc;
        s_DefaultSAXHandler.endElement = DefaultEndElementSAXFunc;
        s_DefaultSAXHandler.characters = DefaultCharactersSAXFunc;
        s_DefaultSAXHandler.error = RaveXMLErrorFunc;
        s_DefaultSAXHandler.initialized = 1;
    }

    xmlSAXHandlerPtr sax = &s_DefaultSAXHandler;
    int ret = 0;
    xmlParserCtxtPtr ctxt;

    ctxt = xmlCreateMemoryParserCtxt(buffer, size);
    if (ctxt == NULL) {
        return false;
    }
    if (ctxt->sax != (xmlSAXHandlerPtr) &xmlDefaultSAXHandler) {
        xmlFree(ctxt->sax);
    }
    ctxt->sax = sax;
    xmlDetectSAX2(ctxt);

    XMLREADERDATA reader(preader, ctxt);
    ctxt->userData = &reader;

    xmlParseDocument(ctxt);

    if (ctxt->wellFormed) {
        ret = 0;
    }
    else {
        if (ctxt->errNo != 0) {
            ret = ctxt->errNo;
        }
        else {
            ret = -1;
        }
    }
    if (sax != NULL) {
        ctxt->sax = NULL;
    }
    if (ctxt->myDoc != NULL) {
        xmlFreeDoc(ctxt->myDoc);
        ctxt->myDoc = NULL;
    }
    xmlFreeParserCtxt(ctxt);

    return ret==0;
}

}

#ifdef _WIN32
const char *strcasestr(const char *s, const char *find)
{
    register char c, sc;
    register size_t len;

    if ((c = *find++) != 0) {
        c = tolower((unsigned char)c);
        len = strlen(find);
        do {
            do {
                if ((sc = *s++) == 0) {
                    return (NULL);
                }
            } while ((char)tolower((unsigned char)sc) != c);
        } while (strnicmp(s, find, len) != 0);
        s--;
    }
    return ((char *) s);
}
#endif

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

PlannerBase::PlannerParameters::PlannerParameters() : XMLReadable("plannerparameters"), _fStepLength(0.04f), _nMaxIterations(0), _sPathOptimizationPlanner("shortcut_linear")
{
    _diffstatefn = subtractstates;
    _neighstatefn = addstates;
    _vXMLParameters.reserve(10);
    _vXMLParameters.push_back("_vinitialconfig");
    _vXMLParameters.push_back("_vgoalconfig");
    _vXMLParameters.push_back("_vconfiglowerlimit");
    _vXMLParameters.push_back("_vconfigupperlimit");
    _vXMLParameters.push_back("_vconfigresolution");
    _vXMLParameters.push_back("_nmaxiterations");
    _vXMLParameters.push_back("_fsteplength");
    _vXMLParameters.push_back("_pathoptimization");
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
    _vConfigLowerLimit.resize(0);
    _vConfigUpperLimit.resize(0);
    _vConfigResolution.resize(0);
    _sPathOptimizationPlanner = "shortcut_linear";
    _sPathOptimizationParameters.resize(0);
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
    O << "<_vconfigresolution>";
    FOREACHC(it, _vConfigResolution) {
        O << *it << " ";
    }
    O << "</_vconfigresolution>" << endl;

    O << "<_nmaxiterations>" << _nMaxIterations << "</_nmaxiterations>" << endl;
    O << "<_fsteplength>" << _fStepLength << "</_fsteplength>" << endl;
    O << "<_pathoptimization planner=\"" << _sPathOptimizationPlanner << "\">" << _sPathOptimizationParameters << "</_pathoptimization>" << endl;
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

    if( name == "_pathoptimization" ) {
        _sslocal.reset(new std::stringstream());
        _sPathOptimizationPlanner="";
        _sPathOptimizationParameters="";
        FOREACHC(itatt,atts) {
            if( itatt->first == "planner" ) {
                _sPathOptimizationPlanner = itatt->second;
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

    if((name=="_vinitialconfig")||(name=="_vgoalconfig")||(name=="_vconfiglowerlimit")||(name=="_vconfigupperlimit")||(name=="_vconfigresolution")||(name=="_nmaxiterations")||(name=="_fsteplength")||(name=="_pathoptimization")) {
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
                if( pdummy->GetFieldName() == "_pathoptimization" ) {
                    _sPathOptimizationParameters = _sslocal->str();
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
    robot->GetActiveDOFResolutions(_vConfigResolution);
    robot->GetActiveDOFValues(vinitialconfig);
    BOOST_ASSERT((int)_vConfigResolution.size()==robot->GetActiveDOF());
}

bool PlannerBase::InitPlan(RobotBasePtr pbase, std::istream& isParameters)
{
    RAVELOG_WARN(str(boost::format("using default planner parameters structure to de-serialize parameters data inside %s, information might be lost!! Please define a InitPlan(robot,stream) function!\n")%GetXMLId()));
    boost::shared_ptr<PlannerParameters> localparams(new PlannerParameters());
    isParameters >> *localparams;
    return InitPlan(pbase,localparams);
}

bool PlannerBase::_OptimizePath(RobotBasePtr probot, TrajectoryBasePtr ptraj)
{
    if( GetParameters()->_sPathOptimizationPlanner.size() == 0 ) {
        return true;
    }
    PlannerBasePtr planner = RaveCreatePlanner(GetEnv(), GetParameters()->_sPathOptimizationPlanner);
    if( !planner ) {
        planner = RaveCreatePlanner(GetEnv(), "shortcut_linear");
        if( !planner ) {
            return false;
        }
    }
    PlannerParametersPtr params(new PlannerParameters());
    params->copy(GetParameters());
    params->_sExtraParameters += GetParameters()->_sPathOptimizationParameters;
    params->_sPathOptimizationPlanner = "";
    params->_sPathOptimizationParameters = "";
    params->_nMaxIterations = 0; // have to reset since path optimizers also use it and new parameters could be in extra parameters
    bool bSuccess = planner->InitPlan(probot, params) && planner->PlanPath(ptraj);

    if( !bSuccess && planner->GetXMLId() != "shortcut_linear") {
        RAVELOG_DEBUG("trying shortcut_linear\n");
        planner = RaveCreatePlanner(GetEnv(), "shortcut_linear");
        if( !!planner ) {
            bSuccess = planner->InitPlan(probot, params) && planner->PlanPath(ptraj);
        }
    }
    return bSuccess;
}

}
