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
#include "md5.h"

namespace OpenRAVE {

#ifdef _DEBUG
RAVE_API DebugLevel g_nDebugLevel = Level_Debug;
#else
RAVE_API DebugLevel g_nDebugLevel = Level_Info;
#endif

RAVE_API void RaveSetDebugLevel(DebugLevel level)
{
    g_nDebugLevel = level;
}

RAVE_API const std::map<PluginType,std::string>& RaveGetInterfaceNamesMap()
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

RAVE_API const std::string& RaveGetInterfaceName(PluginType type)
{
    std::map<PluginType,std::string>::const_iterator it = RaveGetInterfaceNamesMap().find(type);
    if( it == RaveGetInterfaceNamesMap().end() )
        throw openrave_exception("Invalid type specified");
    return it->second;
}

void BaseXMLReader::startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts)
{
    if( __bRecordXMLData ) {
        __sxml << "<" << name << " ";
        FOREACHC(itatt, atts)
            __sxml << itatt->first << "=\"" << itatt->second << "\" ";
        __sxml << ">" << endl;
    }
}

bool BaseXMLReader::endElement(const std::string& name)
{
    if( __bRecordXMLData )
        __sxml << "</" << name << ">" << endl;
    return false;
}

void BaseXMLReader::characters(const std::string& ch)
{
    if( __bRecordXMLData )
        __sxml << ch;
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
    BaseXMLReader::startElement(name,atts);
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
    BaseXMLReader::endElement(name);

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
    BaseXMLReader::startElement(name,atts);
    if( name == _tag )
        ++_numtags;
    else if( !!_preader )
        _preader->startElement(name, atts);
}

bool OneTagReader::endElement(const std::string& name)
{
    BaseXMLReader::endElement(name);

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
    BaseXMLReader::characters(ch);
    if( !!_preader )
        _preader->characters(ch);
}

// PlannerParameters class
PlannerBase::PlannerParameters::PlannerParameters() : XMLReadable("plannerparameters"), _fStepLength(0.04f), _nMaxIterations(0), _bComputeSmoothPath(true)
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
    _samplegoalfn = r._samplegoalfn;
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
    O << "<vinitialconfig>";
    FOREACHC(it, vinitialconfig)
        O << *it << " ";
    O << "</vinitialconfig>" << endl;
    O << "<vgoalconfig>";
    FOREACHC(it, vgoalconfig)
        O << *it << " ";
    O << "</vgoalconfig>" << endl;
    O << "<_vconfiglowerlimit>";
    FOREACHC(it, _vConfigLowerLimit)
        O << *it << " ";
    O << "</_vconfiglowerlimit>" << endl;
    O << "<_vconfigupperlimit>";
    FOREACHC(it, _vConfigUpperLimit)
        O << *it << " ";
    O << "</_vconfigupperlimit>" << endl;
    O << "<_vconfigresolution>";
    FOREACHC(it, _vConfigResolution)
        O << *it << " ";
    O << "</_vconfigresolution>" << endl;
    
    if( !!_tWorkspaceGoal )
        O << "<_tworkspacegoal>" << *_tWorkspaceGoal << "</_tworkspacegoal>" << endl;
    
    O << "<_nmaxiterations>" << _nMaxIterations << "</_nmaxiterations>" << endl;
    O << "<_fsteplength>" << _fStepLength << "</_fsteplength>" << endl;
    O << "<_bcomputesmoothpath>" << _bComputeSmoothPath << "</_bcomputesmoothpath>" << endl;
    
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
    else if( name == "vinitialconfig")
        vinitialconfig = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
    else if( name == "vgoalconfig")
        vgoalconfig = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
    else if( name == "_vconfiglowerlimit")
        _vConfigLowerLimit = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
    else if( name == "_vconfigupperlimit")
        _vConfigUpperLimit = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
    else if( name == "_vconfigresolution")
        _vConfigResolution = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
    else if( name == "_tworkspacegoal") {
        _tWorkspaceGoal.reset(new Transform());
        _ss >> *_tWorkspaceGoal.get();
    }
    else if( name == "_nmaxiterations")
        _ss >> _nMaxIterations;
    else if( name == "_fsteplength")
        _ss >> _fStepLength;
    else if( name == "_bcomputesmoothpath")
        _ss >> _bComputeSmoothPath;
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
        BOOST_ASSERT((int)c0.size()==_robot->GetActiveDOF());
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
        sample.resize(lower.size());
        int dof = lower.size();
        for (int i = 0; i < dof; i++)
            sample[i] = pCurSample[i] + 10.0f*fRadius*(RaveRandomFloat()-0.5f);

        // normalize
        dReal fRatio = max((dReal)1e-5f,fRadius*(0.1f+0.9f*RaveRandomFloat()));
            
        //assert(_robot->ConfigDist(&_vzero[0], &_vSampleConfig[0]) < B+1);
        dReal fDist = _distmetricfn(sample,pCurSample);
        while(fDist > fRatio) {
            for (int i = 0; i < dof; i++)
                sample[i] = 0.5f*pCurSample[i]+0.5f*sample[i];
            fDist = _distmetricfn(sample,pCurSample);
        }
    
        for(int iter = 0; iter < 20; ++iter) {
            while(_distmetricfn(sample, pCurSample) < fRatio ) {
                for (int i = 0; i < dof; i++)
                    sample[i] = 1.2f*sample[i]-0.2f*pCurSample[i];
            }
        }

        pNewSample.resize(lower.size());
        for (int i = 0; i < dof; i++) {
            if( sample[i] < lower[i] )
                pNewSample[i] = lower[i];
            else if( sample[i] > upper[i] )
                pNewSample[i] = upper[i];
            else
                pNewSample[i] = sample[i];
        }

        return true;
    }

 protected:
    RobotBasePtr _robot;
    vector<dReal> lower, upper, range,sample;
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
    robot->GetActiveDOFValues(vinitialconfig);
    BOOST_ASSERT((int)_vConfigResolution.size()==robot->GetActiveDOF());
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
            throw openrave_exception(str(boost::format("error, failed to find </PlannerParameters> in %s")%buf.str()),ORE_InvalidArguments);

        OneTagReader tagreader("plannerparameters", boost::shared_ptr<BaseXMLReader>(&pp,null_deleter()));
        pp._pcurreader.reset();
        LocalXML::ParseXMLData(&tagreader, buf.str().c_str(), -1);
    }

    return I;
}

bool InterfaceBase::Clone(InterfaceBaseConstPtr preference, int cloningoptions)
{
    if( !preference )
        throw openrave_exception("invalid cloning reference",ORE_InvalidArguments);
    __pUserData = preference->__pUserData;
    __strxmlfilename = preference->__strxmlfilename;
    __mapReadableInterfaces = preference->__mapReadableInterfaces;
    return true;
}

ProblemInstance::ProblemInstance(EnvironmentBasePtr penv) : InterfaceBase(PT_ProblemInstance, penv)
{
    RegisterCommand("help",boost::bind(&ProblemInstance::GetCommandHelp,this,_1,_2),
                    "display help message.");
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

bool ProblemInstance::GetCommandHelp(std::ostream& o, std::istream& sinput) const
{
    int maxlen = 0;
    CMDMAP::const_iterator it;
    for(it = __mapCommands.begin(); it != __mapCommands.end(); ++it) {
        if( maxlen  < (int)it->first.size() )
            maxlen = (int)it->first.size();
    }
    
    o << "----------------------------------" << endl
      << GetXMLId() << " Commands:" << endl;
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
    o << "----------------------------------" << endl;
    return true;
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

/// SimpleSensorSystem
SimpleSensorSystem::SimpleXMLReader::SimpleXMLReader(boost::shared_ptr<XMLData> p) : _pdata(p)
{
}

void SimpleSensorSystem::SimpleXMLReader::startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts)
{
    if( !!_pcurreader )
        _pcurreader->startElement(name,atts);
    else if( name != _pdata->GetXMLId() && name != "offsetlink" && name != "id" && name != "sid" && name != "translation" && name != "rotationmat" && name != "rotationaxis" && name != "quat" && name != "pretranslation" && name != "prerotation" && name != "prerotationaxis" && name != "prequat" ) {
        _pcurreader.reset(new DummyXMLReader(name, _pdata->GetXMLId()));
    }
}

bool SimpleSensorSystem::SimpleXMLReader::endElement(const std::string& name)
{
    if( name == _pdata->GetXMLId() )
        return true;

    if( name == "offsetlink" )
        ss >> _pdata->strOffsetLink;
    else if( name == "id" )
        ss >> _pdata->id;
    else if( name == "sid" )
        ss >> _pdata->sid;
    else if( name == "translation" )
        ss >> _pdata->transOffset.trans.x >> _pdata->transOffset.trans.y >> _pdata->transOffset.trans.z;
    else if( name == "rotationmat" ) {
        TransformMatrix m;
        ss >> m.m[0] >> m.m[1] >> m.m[2] >> m.m[4] >> m.m[5] >> m.m[6] >> m.m[8] >> m.m[9] >> m.m[10];
        _pdata->transOffset.rot = Transform(m).rot;
    }
    else if( name == "rotationaxis" ) {
        Vector axis; dReal fang;
        ss >> axis.x >> axis.y >> axis.z >> fang;
        _pdata->transOffset.rotfromaxisangle(axis.normalize3(),fang*(PI/180.0f));
    }
    else if( name == "quat" )
        ss >> _pdata->transOffset.rot;
    else if( name == "pretranslation")
        ss >> _pdata->transPreOffset.trans.x >> _pdata->transPreOffset.trans.y >> _pdata->transPreOffset.trans.z;
    else if( name == "prerotationmat") {
        TransformMatrix m;
        ss >> m.m[0] >> m.m[1] >> m.m[2] >> m.m[4] >> m.m[5] >> m.m[6] >> m.m[8] >> m.m[9] >> m.m[10];
        _pdata->transPreOffset.rot = Transform(m).rot;
    }
    else if( name == "prerotationaxis") {
        Vector axis; dReal fang;
        ss >> axis.x >> axis.y >> axis.z >> fang;
        _pdata->transPreOffset.rotfromaxisangle(axis,fang*(PI/180.0f));
    }
    else if( name == "prequat")
        ss >> _pdata->transPreOffset.rot;
    else
        RAVELOG_WARNA(str(boost::format("bad tag: %s")%name));

    if( !ss )
        RAVELOG_WARNA(str(boost::format("error parsing %s\n")%name));

    return false;
}

void SimpleSensorSystem::SimpleXMLReader::characters(const std::string& ch)
{
    if( !!_pcurreader )
        _pcurreader->characters(ch);
    else {
        ss.clear();
        ss.str(ch);
    }
}

BaseXMLReaderPtr SimpleSensorSystem::CreateXMLReaderId(const string& xmlid, InterfaceBasePtr ptr, const std::list<std::pair<std::string,std::string> >& atts)
{
    return BaseXMLReaderPtr(new SimpleXMLReader(boost::shared_ptr<XMLData>(new XMLData(xmlid))));
}

boost::shared_ptr<void> SimpleSensorSystem::RegisterXMLReaderId(EnvironmentBasePtr penv, const string& xmlid)
{
    return penv->RegisterXMLReader(PT_KinBody,xmlid, boost::bind(&SimpleSensorSystem::CreateXMLReaderId,xmlid, _1,_2));
}

SimpleSensorSystem::SimpleSensorSystem(const std::string& xmlid, EnvironmentBasePtr penv) : SensorSystemBase(penv), _expirationtime(2000000), _bShutdown(false), _threadUpdate(boost::bind(&SimpleSensorSystem::_UpdateBodiesThread,this))
{
    _xmlid = xmlid;
    std::transform(_xmlid.begin(), _xmlid.end(), _xmlid.begin(), ::tolower);
}

SimpleSensorSystem::~SimpleSensorSystem()
{
    Reset();
    _bShutdown = true;
    _threadUpdate.join();
}

void SimpleSensorSystem::Reset()
{
    boost::mutex::scoped_lock lock(_mutex);
    _mapbodies.clear();        
}

void SimpleSensorSystem::AddRegisteredBodies(const std::vector<KinBodyPtr>& vbodies)
{
    // go through all bodies in the environment and check for mocap data
    FOREACHC(itbody, vbodies) {
        boost::shared_ptr<XMLData> pmocapdata = boost::dynamic_pointer_cast<XMLData>((*itbody)->GetReadableInterface(_xmlid));
        if( !!pmocapdata ) {
            KinBody::ManageDataPtr p = AddKinBody(*itbody, pmocapdata);
            if( !!p )
                p->Lock(true);
        }
    }
}

KinBody::ManageDataPtr SimpleSensorSystem::AddKinBody(KinBodyPtr pbody, XMLReadableConstPtr _pdata)
{
    BOOST_ASSERT(pbody->GetEnv()==GetEnv());
    boost::shared_ptr<XMLData const> pdata = boost::dynamic_pointer_cast<XMLData const>(_pdata);
    if( !pdata ) {
        pdata = boost::dynamic_pointer_cast<XMLData const>(pbody->GetReadableInterface(_xmlid));
        if( !pdata ) {
            RAVELOG_ERRORA(str(boost::format("failed to find mocap data for body %s\n")%pbody->GetName()));
            return KinBody::ManageDataPtr();
        }
    }

    boost::mutex::scoped_lock lock(_mutex);
    if( _mapbodies.find(pbody->GetNetworkId()) != _mapbodies.end() ) {
        RAVELOG_WARNA(str(boost::format("body %s already added\n")%pbody->GetName()));
        return KinBody::ManageDataPtr();
    }
    
    boost::shared_ptr<BodyData> b = CreateBodyData(pbody, pdata);
    b->lastupdated = GetMicroTime();
    _mapbodies[pbody->GetNetworkId()] = b;
    RAVELOG_DEBUGA(str(boost::format("system adding body %s, total: %d\n")%pbody->GetName()%_mapbodies.size()));
    return b;
}

bool SimpleSensorSystem::RemoveKinBody(KinBodyPtr pbody)
{
    boost::mutex::scoped_lock lock(_mutex);
    bool bSuccess = _mapbodies.erase(pbody->GetNetworkId())>0;
    RAVELOG_DEBUGA(str(boost::format("system removing body %s %s\n")%pbody->GetName()%(bSuccess?"succeeded":"failed")));
    return bSuccess;
}

bool SimpleSensorSystem::IsBodyPresent(KinBodyPtr pbody)
{
    boost::mutex::scoped_lock lock(_mutex);
    return _mapbodies.find(pbody->GetNetworkId()) != _mapbodies.end();
}

bool SimpleSensorSystem::EnableBody(KinBodyPtr pbody, bool bEnable)
{
    boost::mutex::scoped_lock lock(_mutex);
    BODIES::iterator it = _mapbodies.find(pbody->GetNetworkId());
    if( it == _mapbodies.end() ) {
        RAVELOG_WARNA("trying to %s body %s that is not in system\n", bEnable?"enable":"disable", pbody->GetName().c_str());
        return false;
    }

    it->second->bEnabled = bEnable;
    return true;
}

bool SimpleSensorSystem::SwitchBody(KinBodyPtr pbody1, KinBodyPtr pbody2)
{
    //boost::mutex::scoped_lock lock(_mutex);
    BODIES::iterator it = _mapbodies.find(pbody1->GetNetworkId());
    boost::shared_ptr<BodyData> pb1,pb2;
    if( it != _mapbodies.end() )
        pb1 = it->second;
    it = _mapbodies.find(pbody2->GetNetworkId());
    if( it != _mapbodies.end() )
        pb2 = it->second;

    if( !pb1 || !pb2 )
        return false;

    if( !!pb1 )
        pb1->SetBody(pbody2);
    if( !!pb2 )
        pb2->SetBody(pbody1);

    return true;
}

boost::shared_ptr<SimpleSensorSystem::BodyData> SimpleSensorSystem::CreateBodyData(KinBodyPtr pbody, boost::shared_ptr<XMLData const> pdata)
{
    boost::shared_ptr<XMLData> pnewdata(new XMLData(_xmlid));
    pnewdata->copy(pdata);
    return boost::shared_ptr<BodyData>(new BodyData(boost::static_pointer_cast<SimpleSensorSystem>(shared_from_this()),pbody, pnewdata));
}

void SimpleSensorSystem::_UpdateBodies(list<SimpleSensorSystem::SNAPSHOT>& listbodies)
{
    // assume mutex is already locked
    uint64_t curtime = GetMicroTime();
    if( listbodies.size() > 0 ) {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());

        FOREACH(it, listbodies) {
            assert( it->first->IsEnabled() );

            KinBody::LinkPtr plink = it->first->GetOffsetLink();
            if( !plink )
                continue;

            // transform with respect to offset link
            TransformMatrix tlink = plink->GetTransform();
            TransformMatrix tbase = plink->GetParent()->GetTransform();
            TransformMatrix toffset = tbase * tlink.inverse() * it->first->_initdata->transOffset;
            TransformMatrix tfinal = toffset * it->second*it->first->_initdata->transPreOffset;
            
            plink->GetParent()->SetTransform(tfinal);
            it->first->lastupdated = curtime;
            it->first->tnew = it->second;
            
            //RAVELOG_DEBUGA("%f %f %f\n", tfinal.trans.x, tfinal.trans.y, tfinal.trans.z);
            
            if( !it->first->IsPresent() )
                RAVELOG_VERBOSEA(str(boost::format("updating body %s\n")%plink->GetParent()->GetName()));
            it->first->bPresent = true;
        }
    }

    BODIES::iterator itbody = _mapbodies.begin();
    while(itbody != _mapbodies.end()) {
        if( curtime-itbody->second->lastupdated > _expirationtime ) {
            KinBody::LinkPtr plink = itbody->second->GetOffsetLink();

            if( !itbody->second->IsLocked() ) {                
                if( !!plink ) {
                    //RAVELOG_VERBOSEA(str(boost::format("object %s expired %fs\n")%plink->GetParent()->GetName()*((curtime-itbody->second->lastupdated)*1e-6f)));
                    GetEnv()->RemoveKinBody(plink->GetParent());
                }
                _mapbodies.erase(itbody++);
                continue;
            }
                
            if( itbody->second->IsPresent() && !!plink )
                RAVELOG_VERBOSEA(str(boost::format("body %s not present\n")%plink->GetParent()->GetName()));
            itbody->second->bPresent = false;
        }

        ++itbody;
    }
}

void SimpleSensorSystem::_UpdateBodiesThread()
{
    list< SNAPSHOT > listbodies;

    while(!_bShutdown) {
        {
            boost::mutex::scoped_lock lock(_mutex);
            _UpdateBodies(listbodies);
        }
        Sleep(10); // 10ms
    }
}

RAVE_API void RaveInitRandomGeneration(uint32_t seed)
{
    init_genrand(seed);
}

RAVE_API uint32_t RaveRandomInt()
{
    return genrand_int32();
}

RAVE_API void RaveRandomInt(int n, std::vector<int>& v)
{
    v.resize(n);
    FOREACH(it, v) *it = genrand_int32();
}

RAVE_API float RaveRandomFloat()
{
    return (float)genrand_real1();
}

RAVE_API void RaveRandomFloat(int n, std::vector<float>& v)
{
    v.resize(n);
    FOREACH(it, v) *it = (float)genrand_real1();
}

RAVE_API double RaveRandomDouble()
{
    return genrand_res53();
}
 
RAVE_API void RaveRandomDouble(int n, std::vector<double>& v)
{
    v.resize(n);
    FOREACH(it, v) *it = genrand_res53();
}

std::string GetMD5HashString(const std::string& s)
{
    if( s.size() == 0 )
        return "";

    md5_state_t state;
	md5_byte_t digest[16];
	
	md5_init(&state);
	md5_append(&state, (const md5_byte_t *)s.c_str(), s.size());
	md5_finish(&state, digest);
    string hex_output;
    hex_output.resize(32);
    for (int di = 0; di < 16; ++di) {
        int n = (digest[di]&0xf);
        hex_output[2*di+1] = n > 9 ? ('a'+n-10) : ('0'+n);
        n = (digest[di]&0xf0)>>4;
        hex_output[2*di+0] = n > 9 ? ('a'+n-10) : ('0'+n);
    }
    return hex_output;
}

std::string GetMD5HashString(const std::vector<uint8_t>& v)
{
    if( v.size() == 0 )
        return "";

    md5_state_t state;
	md5_byte_t digest[16];
	
	md5_init(&state);
	md5_append(&state, (const md5_byte_t *)&v[0], v.size());
	md5_finish(&state, digest);
    string hex_output;
    hex_output.resize(32);
    for (int di = 0; di < 16; ++di) {
        int n = (digest[di]&0xf);
        hex_output[2*di+0] = n > 9 ? ('a'+n-10) : ('0'+n);
        n = (digest[di]&0xf0)>>4;
        hex_output[2*di+1] = n > 9 ? ('a'+n-10) : ('0'+n);
    }
    return hex_output;
}

} // end namespace OpenRAVE
