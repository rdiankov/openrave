// Copyright (C) 2006-2009 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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

#include <streambuf>
#include "mt19937ar.h"

namespace OpenRAVE {

#ifdef _DEBUG
DebugLevel g_nDebugLevel = Level_Debug;
#else
DebugLevel g_nDebugLevel = Level_Info;
#endif

const std::map<PluginType,std::string>& RaveGetInterfaceNamesMap()
{
    static map<PluginType,string> m;
    if( m.size() == 0 ) {
        m[PT_Planner] = "planner";
        m[PT_Robot] = "robot";
        m[PT_SensorSystem] = "sensorsystem";
        m[PT_Controller] = "controller";
        m[PT_ProblemInstance] = "probleminstance";
        m[PT_InverseKinematicsSolver] = "inversekinematicssolver";
        m[PT_KinBody] = "kinbody";
        m[PT_PhysicsEngine] = "physicsengine";
        m[PT_Sensor] = "sensor";
        m[PT_CollisionChecker] = "collisionchecker";
        m[PT_Trajectory] = "trajectory";
        m[PT_Viewer] = "viewer";
    }
    return m;
}

const std::string& RaveGetInterfaceName(PluginType type)
{
    std::map<PluginType,std::string>::const_iterator it = RaveGetInterfaceNamesMap().find(type);
    if( it == RaveGetInterfaceNamesMap().end() )
        throw openrave_exception("Invalid type specified");
    return it->second;
}

// Dummy Reader
DummyXMLReader::DummyXMLReader(const std::string& pfieldname, const std::string& pparentname)
{
    _fieldname = pfieldname;
    _parentname = pparentname;
    _parentname += ":";
    _parentname += _fieldname;
    RAVELOG_DEBUGA("unknown xml field: %s\n", _parentname.c_str());
}

void DummyXMLReader::startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts)
{
    if( !!_pcurreader ) {
        _pcurreader->startElement(name, atts);
    }
    else {
        // create a new parser
        _pcurreader.reset(new DummyXMLReader(name, _parentname));
    }
}
    
bool DummyXMLReader::endElement(const std::string& name)
{
    if( !!_pcurreader ) {
        if( _pcurreader->endElement(name) )
            _pcurreader.reset();
    }
    else if( name == _fieldname )
        return true;
    else {
        RAVELOG_ERRORA(str(boost::format("invalid xml tag %s\n")%name));
        return true;
    }

    return false;
}

// OneTag Reader
OneTagReader::OneTagReader(const std::string& tag, BaseXMLReaderPtr preader) : _preader(preader), _tag(tag), _numtags(0)
{
}

void OneTagReader::startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts)
{
    if( name == _tag )
        ++_numtags;
    else if( !!_preader )
        _preader->startElement(name, atts);
}

bool OneTagReader::endElement(const std::string& name)
{
    if( name == _tag ) {
        --_numtags;
        if( _numtags <= 0 )
            return true;
    }
    else if( !!_preader )
        return _preader->endElement(name);

    return false;
}

void OneTagReader::characters(const std::string& ch)
{
    if( !!_preader )
        _preader->characters(ch);
}

// PlannerParameters class
PlannerBase::PlannerParameters::PlannerParameters() : XMLReadable("plannerparameters"), _fStepLength(0.04f), _nMaxIterations(0)
{
}

PlannerBase::PlannerParameters::PlannerParameters(const PlannerParameters& r) : XMLReadable("plannerparameters")
{
    *this = r;
}

PlannerBase::PlannerParameters& PlannerBase::PlannerParameters::operator=(const PlannerBase::PlannerParameters& r)
{
    // reset
    _costfn = r._costfn;
    _goalfn = r._goalfn;
    _distmetricfn = r._distmetricfn;
    _constraintfn = r._constraintfn;
    _samplefn = r._samplefn;
    _sampleneighfn = r._sampleneighfn;
    _setstatefn = r._setstatefn;
    _getstatefn = r._getstatefn;
    
    _tWorkspaceGoal.reset();
    vinitialconfig.resize(0);
    vgoalconfig.resize(0);
    _vConfigLowerLimit.resize(0);
    _vConfigUpperLimit.resize(0);
    _vConfigResolution.resize(0);

    // transfer data
    std::stringstream ss;
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
    O << "<initialconfig>";
    FOREACHC(it, vinitialconfig)
        O << *it << " ";
    O << "</initialconfig>" << endl;
    O << "<goalconfig>";
    FOREACHC(it, vgoalconfig)
        O << *it << " ";
    O << "</goalconfig>" << endl;
    O << "<configlowerlimit>";
    FOREACHC(it, _vConfigLowerLimit)
        O << *it << " ";
    O << "</configlowerlimit>" << endl;
    O << "<configupperlimit>";
    FOREACHC(it, _vConfigUpperLimit)
        O << *it << " ";
    O << "</configupperlimit>" << endl;
    O << "<configresolution>";
    FOREACHC(it, _vConfigResolution)
        O << *it << " ";
    O << "</configresolution>" << endl;
    
    if( !!_tWorkspaceGoal )
        O << "<workspacegoal>" << *_tWorkspaceGoal << "</workspacegoal>" << endl;
    
    O << "<maxiterations>" << _nMaxIterations << "</maxiterations>" << endl;
    O << "<steplength>" << _fStepLength << "</steplength>" << endl;
    
    return !!O;
}

void PlannerBase::PlannerParameters::startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts)
{
    if( !!_pcurreader )
        _pcurreader->startElement(name, atts);
}
        
bool PlannerBase::PlannerParameters::endElement(const std::string& name)
{
    if( !!_pcurreader ) {
        if( _pcurreader->endElement(name) )
            _pcurreader.reset();
    }
    else if( name == "initialconfig")
        vinitialconfig = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
    else if( name == "goalconfig")
        vgoalconfig = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
    else if( name == "configlowerlimit")
        _vConfigLowerLimit = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
    else if( name == "configupperlimit")
        _vConfigUpperLimit = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
    else if( name == "configresolution")
        _vConfigResolution = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
    else if( name == "workspacegoal") {
        _tWorkspaceGoal.reset(new Transform());
        _ss >> *_tWorkspaceGoal.get();
    }
    else if( name == "maxiterations")
        _ss >> _nMaxIterations;
    else if( name == "steplength")
        _ss >> _fStepLength;
    else
        _pcurreader.reset(new DummyXMLReader(name,GetXMLId()));

    return false;
}

void PlannerBase::PlannerParameters::characters(const std::string& ch)
{
    _ss.clear();
    _ss.str(ch);
}

std::ostream& operator<<(std::ostream& O, const PlannerBase::PlannerParameters& v)
{
    O << "<" << v.GetXMLId() << ">" << endl;
    v.serialize(O);
    O << "</" << v.GetXMLId() << ">" << endl;
    return O;
}

class SimpleDistMetric
{
 public:
 SimpleDistMetric(RobotBasePtr robot) : _robot(robot)
    {
        dReal ftransweight = 2;
        weights.resize(0);
        vector<int>::const_iterator it;
        FORIT(it, _robot->GetActiveJointIndices()) weights.push_back(_robot->GetJointWeight(*it));
        if( _robot->GetAffineDOF() & RobotBase::DOF_X ) weights.push_back(ftransweight);
        if( _robot->GetAffineDOF() & RobotBase::DOF_Y ) weights.push_back(ftransweight);
        if( _robot->GetAffineDOF() & RobotBase::DOF_Z ) weights.push_back(ftransweight);
        if( _robot->GetAffineDOF() & RobotBase::DOF_RotationAxis ) weights.push_back(ftransweight);
        else if( _robot->GetAffineDOF() & RobotBase::DOF_RotationQuat ) {
            weights.push_back(0.4f);
            weights.push_back(0.4f);
            weights.push_back(0.4f);
            weights.push_back(0.4f);
        }
    }

    virtual dReal Eval(const std::vector<dReal>& c0, const std::vector<dReal>& c1)
    {
        dReal out = 0;
        for(int i=0; i < _robot->GetActiveDOF(); i++)
            out += weights[i] * (c0[i]-c1[i])*(c0[i]-c1[i]);
            
        return RaveSqrt(out);
    }

 protected:
    RobotBasePtr _robot;
    vector<dReal> weights;
};

class SimpleSampleFunction
{
public:
    SimpleSampleFunction(RobotBasePtr robot, const boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)>& distmetricfn) : _robot(robot), _distmetricfn(distmetricfn) {
        _robot->GetActiveDOFLimits(lower, upper);
        range.resize(lower.size());
        for(int i = 0; i < (int)range.size(); ++i)
            range[i] = upper[i] - lower[i];
    }
    virtual bool Sample(vector<dReal>& pNewSample) {
        pNewSample.resize(lower.size());
        for (size_t i = 0; i < lower.size(); i++)
            pNewSample[i] = lower[i] + RaveRandomFloat()*range[i];
        return true;
    }

    virtual bool SampleNeigh(vector<dReal>& pNewSample, const vector<dReal>& pCurSample, dReal fRadius)
    {
        BOOST_ASSERT(pCurSample.size()==lower.size());
        pNewSample.resize(lower.size());
        int dof = lower.size();
        for (int i = 0; i < dof; i++)
            pNewSample[i] = pCurSample[i] + 10.0f*fRadius*(RaveRandomFloat()-0.5f);

        // normalize
        dReal fRatio = fRadius*RaveRandomFloat();
            
        //assert(_robot->ConfigDist(&_vzero[0], &_vSampleConfig[0]) < B+1);
        while(_distmetricfn(pNewSample,pCurSample) > fRatio ) {
            for (int i = 0; i < dof; i++)
                pNewSample[i] = 0.5f*pCurSample[i]+0.5f*pNewSample[i];
        }
            
        while(_distmetricfn(pNewSample, pCurSample) < fRatio ) {
            for (int i = 0; i < dof; i++)
                pNewSample[i] = 1.2f*pNewSample[i]-0.2f*pCurSample[i];
        }

        for (int i = 0; i < dof; i++) {
            if( pNewSample[i] < lower[i] )
                pNewSample[i] = lower[i];
            else if( pNewSample[i] > upper[i] )
                pNewSample[i] = upper[i];
        }

        return true;
    }

 protected:
    RobotBasePtr _robot;
    vector<dReal> lower, upper, range;
    boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)> _distmetricfn;
};


void PlannerBase::PlannerParameters::SetRobotActiveJoints(RobotBasePtr robot)
{
    _distmetricfn = boost::bind(&SimpleDistMetric::Eval,boost::shared_ptr<SimpleDistMetric>(new SimpleDistMetric(robot)),_1,_2);
    boost::shared_ptr<SimpleSampleFunction> defaultsamplefn(new SimpleSampleFunction(robot,_distmetricfn));
    _samplefn = boost::bind(&SimpleSampleFunction::Sample,defaultsamplefn,_1);
    _sampleneighfn = boost::bind(&SimpleSampleFunction::SampleNeigh,defaultsamplefn,_1,_2,_3);
    _setstatefn = boost::bind(&RobotBase::SetActiveDOFValues,robot,_1,false);
    _getstatefn = boost::bind(&RobotBase::GetActiveDOFValues,robot,_1);
    robot->GetActiveDOFLimits(_vConfigLowerLimit,_vConfigUpperLimit);
    robot->GetActiveDOFResolutions(_vConfigResolution);
    BOOST_ASSERT(_vConfigResolution.size()>0);
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
		if ((sc = *s++) == 0)
		    return (NULL);
	    } while ((char)tolower((unsigned char)sc) != c);
	} while (strnicmp(s, find, len) != 0);
	s--;
    }
    return ((char *) s);
}
#endif

#define LIBXML_SAX1_ENABLED
#include <libxml/globals.h>
#include <libxml/xmlerror.h>
#include <libxml/parser.h>
#include <libxml/parserInternals.h> // only for xmlNewInputFromFile()
#include <libxml/tree.h>

#include <libxml/debugXML.h>
#include <libxml/xmlmemory.h>

namespace LocalXML {

void RaveXMLErrorFunc(void *ctx, const char *msg, ...)
{
    va_list args;

    va_start(args, msg);
    RAVELOG_ERRORA("XML Parse error: ");
    wchar_t wfmt[200];
    swprintf(wfmt,200,L"%s",msg);
    vwprintf(wfmt,args);
    va_end(args);
}

struct XMLREADERDATA
{
    XMLREADERDATA(BaseXMLReader* preader, xmlParserCtxtPtr ctxt) : _preader(preader), _ctxt(ctxt) {}
    BaseXMLReader* _preader;
    xmlParserCtxtPtr _ctxt;
};

void DefaultStartElementSAXFunc(void *ctx, const xmlChar *name, const xmlChar **atts)
{
    std::list<std::pair<std::string,std::string> > listatts;
    if( atts != NULL ) {
        for (int i = 0;(atts[i] != NULL);i+=2) {
            listatts.push_back(make_pair(string((const char*)atts[i]),string((const char*)atts[i+1])));
            std::transform(listatts.back().first.begin(), listatts.back().first.end(), listatts.back().first.begin(), ::tolower);
        }
    }

    string s = (const char*)name;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    ((XMLREADERDATA*)ctx)->_preader->startElement(s, listatts);
}

void DefaultEndElementSAXFunc(void *ctx, const xmlChar *name)
{
    XMLREADERDATA* data = (XMLREADERDATA*)ctx;
    string s = (const char*)name;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    if( data->_preader->endElement(s) ) {
        //RAVEPRINT(L"%s size read %d\n", name, data->_ctxt->input->consumed);
        xmlStopParser(data->_ctxt);
    }
}

void DefaultCharactersSAXFunc(void *ctx, const xmlChar *ch, int len)
{
    ((XMLREADERDATA*)ctx)->_preader->characters(string((const char*)ch, len));
}

static bool xmlDetectSAX2(xmlParserCtxtPtr ctxt)
{
    if (ctxt == NULL)
        return false;
#ifdef LIBXML_SAX1_ENABLED
    if ((ctxt->sax) &&  (ctxt->sax->initialized == XML_SAX2_MAGIC) &&
        ((ctxt->sax->startElementNs != NULL) ||
         (ctxt->sax->endElementNs != NULL))) ctxt->sax2 = 1;
#else
    ctxt->sax2 = 1;
#endif /* LIBXML_SAX1_ENABLED */

    ctxt->str_xml = xmlDictLookup(ctxt->dict, BAD_CAST "xml", 3);
    ctxt->str_xmlns = xmlDictLookup(ctxt->dict, BAD_CAST "xmlns", 5);
    ctxt->str_xml_ns = xmlDictLookup(ctxt->dict, XML_XML_NAMESPACE, 36);
    if ((ctxt->str_xml==NULL) || (ctxt->str_xmlns==NULL) || 
        (ctxt->str_xml_ns == NULL)) {
        return false;
    }

    return true;
}

bool ParseXMLData(BaseXMLReader* preader, const char* buffer, int size)
{
    static xmlSAXHandler s_DefaultSAXHandler = {0};

    if( size <= 0 )
        size = strlen(buffer);
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
    if (ctxt == NULL) return false;
    if (ctxt->sax != (xmlSAXHandlerPtr) &xmlDefaultSAXHandler)
        xmlFree(ctxt->sax);
    ctxt->sax = sax;
    xmlDetectSAX2(ctxt);

    XMLREADERDATA reader(preader, ctxt);
    ctxt->userData = &reader;
    
    xmlParseDocument(ctxt);
    
    if (ctxt->wellFormed)
        ret = 0;
    else {
        if (ctxt->errNo != 0)
            ret = ctxt->errNo;
        else
            ret = -1;
    }
    if (sax != NULL)
        ctxt->sax = NULL;
    if (ctxt->myDoc != NULL) {
        xmlFreeDoc(ctxt->myDoc);
        ctxt->myDoc = NULL;
    }
    xmlFreeParserCtxt(ctxt);
    
    return ret==0;
}

}

std::istream& operator>>(std::istream& I, PlannerBase::PlannerParameters& pp)
{
    if( !!I) {
        stringbuf buf;
        stringstream::streampos pos = I.tellg();
        I.get(buf, 0); // get all the data

        const char* p = strcasestr(buf.str().c_str(), "</PlannerParameters>");

        if( p != NULL ) {
            I.clear();
            I.seekg((size_t)pos+((p-buf.str().c_str())+20));
        }
        else
            throw openrave_exception(str(boost::format("error, failed to find </PlannerParameters> in %s\n")%buf.str()),ORE_InvalidArguments);

        OneTagReader tagreader("plannerparameters", boost::shared_ptr<BaseXMLReader>(&pp,null_deleter()));
        pp._pcurreader.reset();
        LocalXML::ParseXMLData(&tagreader, buf.str().c_str(), -1);
    }

    return I;
}

bool ProblemInstance::SendCommand(ostream& sout, istream& sinput)
{
    string cmd;
    sinput >> cmd;
    if( !sinput )
        throw openrave_exception("invalid argument",ORE_InvalidArguments);
    
    CMDMAP::iterator it = __mapCommands.find(cmd);
    if( it == __mapCommands.end() )
        throw openrave_exception(str(boost::format("failed to find command %s in problem %s\n")%cmd.c_str()%GetXMLId()),ORE_CommandNotSupported);
    if( !it->second.fn(sout,sinput) ) {
        RAVELOG_DEBUGA("command failed in problem %s: %s\n", GetXMLId().c_str(), cmd.c_str());
        return false;
    }
    return true;
}

void ProblemInstance::RegisterCommand(const std::string& cmdname, CommandFn fncmd, const std::string& strhelp)
{
    if( cmdname.size() == 0 || __mapCommands.find(cmdname) != __mapCommands.end() )
        throw openrave_exception(str(boost::format("command %s already registered")%cmdname));
    __mapCommands[cmdname] = COMMAND(fncmd, strhelp);
}

void ProblemInstance::DeleteCommand(const std::string& cmdname)
{
    CMDMAP::iterator it = __mapCommands.find(cmdname);
    if( it != __mapCommands.end() )
        __mapCommands.erase(it);
}

const ProblemInstance::CMDMAP& ProblemInstance::GetCommands() const
{
    return __mapCommands;
}

void ProblemInstance::GetCommandHelp(std::ostream& o) const
{
    int maxlen = 0;
    CMDMAP::const_iterator it;
    for(it = __mapCommands.begin(); it != __mapCommands.end(); ++it) {
        if( maxlen  < (int)it->first.size() )
            maxlen = (int)it->first.size();
    }
    
    for(it = __mapCommands.begin(); it != __mapCommands.end(); ++it) {
        // search for all new lines
        std::string::size_type pos = 0, newpos=0;
        while( pos < it->second.help.size() ) {

            newpos = it->second.help.find('\n', pos);
            
            std::string::size_type n = newpos == std::string::npos ? it->second.help.size()-pos : (newpos-pos);

            if( pos == 0 )
                o << std::setw(maxlen) << std::left << it->first << " - " << it->second.help.substr(pos, n) << std::endl;
            else
                o << std::setw(maxlen+3) << std::setfill(' ') << " " << it->second.help.substr(pos, n) << std::endl;

            if( newpos == std::string::npos )
                break;
            
            pos = newpos+1;
        }
    }
}

bool SensorBase::LaserSensorData::serialize(std::ostream& O) const
{
    RAVELOG_WARNA("LaserSensorData XML serialization not implemented\n");
    return true;
}

bool SensorBase::CameraSensorData::serialize(std::ostream& O) const
{
    RAVELOG_WARNA("CameraSensorData XML serialization not implemented\n");
    return true;
}

void RaveInitRandomGeneration(uint32_t seed)
{
    init_genrand(seed);
}

uint32_t RaveRandomInt()
{
    return genrand_int32();
}

void RaveRandomInt(int n, std::vector<int>& v)
{
    v.resize(n);
    FOREACH(it, v) *it = genrand_int32();
}

float RaveRandomFloat()
{
    return (float)genrand_real1();
}

void RaveRandomFloat(int n, std::vector<float>& v)
{
    v.resize(n);
    FOREACH(it, v) *it = (float)genrand_real1();
}

double RaveRandomDouble()
{
    return genrand_res53();
}
 
void RaveRandomDouble(int n, std::vector<double>& v)
{
    v.resize(n);
    FOREACH(it, v) *it = genrand_res53();
}

} // end namespace OpenRAVE
