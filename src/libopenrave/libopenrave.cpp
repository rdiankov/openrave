// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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

namespace OpenRAVE {


#ifdef _DEBUG
DebugLevel g_nDebugLevel = Level_Debug;
#else
DebugLevel g_nDebugLevel = Level_Info;
#endif

// Dummy Reader
DummyXMLReader::DummyXMLReader(const char* pfieldname, const char* pparentname)
{
    assert( pfieldname != NULL );
    _fieldname = pfieldname;
    _pcurreader = NULL;

    if( pparentname != NULL )
        _parentname = pparentname;
    _parentname += ":";
    _parentname += _fieldname;

    RAVELOG_DEBUGA("unknown xml field: %s\n", _parentname.c_str());
}

void* DummyXMLReader::Release()
{
  return NULL;
}

void DummyXMLReader::startElement(void *ctx, const char *name, const char **atts)
{
    if( _pcurreader != NULL ) {
        _pcurreader->startElement(ctx, name, atts);
    }
    else {
        // create a new parser
        _pcurreader = new DummyXMLReader(name, _parentname.c_str());
    }
}
    
bool DummyXMLReader::endElement(void *ctx, const char *name)
{
    if( _pcurreader != NULL ) {
        if( _pcurreader->endElement(ctx, name) ) {
            delete _pcurreader; _pcurreader = NULL;
        }
    }
    else if( stricmp(name, _fieldname.c_str()) == 0 ) {
        // end
        return true;
    }
    else {
        assert(0);
    }

    return false;
}

// OneTag Reader
OneTagReader::OneTagReader(string tag, BaseXMLReader* preader) : _preader(preader), _tag(tag), _numtags(0)
{
}

void* OneTagReader::Release()
{
  return _preader;
}

void OneTagReader::startElement(void *ctx, const char *name, const char **atts)
{
    if( stricmp(name, _tag.c_str()) == 0 )
        ++_numtags;
    else if( _preader != NULL )
        _preader->startElement(ctx, name, atts);
}

bool OneTagReader::endElement(void *ctx, const char *name)
{
    if( stricmp(name, _tag.c_str()) == 0 ) {
        --_numtags;
        if( _numtags <= 0 )
            return true;
    }
    else if( _preader != NULL )
        return _preader->endElement(ctx, name);

    return false;
}

void OneTagReader::characters(void *ctx, const char *ch, int len)
{
    if( _preader != NULL )
        _preader->characters(ctx, ch, len);
}

// PlannerParameters class
PlannerBase::PlannerParameters::PlannerParameters(const PlannerParameters& r)
{
    *this = r;
}

PlannerBase::PlannerParameters& PlannerBase::PlannerParameters::operator=(const PlannerBase::PlannerParameters& r)
{
    // reset
    pcostfn = r.pcostfn;
    pgoalfn = r.pgoalfn;
    pdistmetric = r.pdistmetric;
    pconstraintfn = r.pconstraintfn;
    pSampleFn = r.pSampleFn;
    pConfigState = r.pConfigState;
    bHasWorkspaceGoal = false;
    vinitialconfig.resize(0);
    vgoalconfig.resize(0);
    vParameters.resize(0);
    vnParameters.resize(0);
    
    // transfer data
    std::stringstream ss;
    ss << r;
    ss >> *this;
    return *this;
}

void PlannerBase::PlannerParameters::copy(const PlannerParameters& r)
{
    *this = r;
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

    if( bHasWorkspaceGoal )
        O << "<workspacegoal>" << tWorkspaceGoal << "</workspacegoal>" << endl;
    
    O << "<maxiterations>" << nMaxIterations << "</maxiterations>" << endl;
    
    O << "<parameters>";
    FOREACHC(it, vParameters)
        O << *it << " ";
    O << "</parameters>" << endl;
    O << "<intparameters>";
    FOREACHC(it, vnParameters)
        O << *it << " ";
    O << "</intparameters>" << endl;

    if( pcostfn != NULL ) {
        O << "<costfunction>";
        pcostfn->serialize(O);
        O << "</costfunction>" << endl;
    }
    if( pgoalfn != NULL ) {
        O << "<goalfunction>";
        pgoalfn->serialize(O);
        O << "</goalfunction>" << endl;
    }
    if( pdistmetric != NULL ) {
        O << "<distmetric>";
        pdistmetric->serialize(O);
        O << "</distmetric>" << endl;
    }
    if( pconstraintfn != NULL ) {
        O << "<constraintfunction>";
        pconstraintfn->serialize(O);
        O << "</constraintfunction>" << endl;
    }
    if( pSampleFn != NULL ) {
        O << "<samplerfunction>";
        pSampleFn->serialize(O);
        O << "</samplerfunction>" << endl;
    }
    if( pConfigState != NULL ) {
        O << "<configurationstate>";
        pConfigState->serialize(O);
        O << "</configurationstate>" << endl;
    }
    
    return !!O;
}

void PlannerBase::PlannerParameters::startElement(void *ctx, const char *name, const char **atts)
{
    if( _pcurreader != NULL ) {
        _pcurreader->startElement(ctx, name, atts);
    }
}
        
bool PlannerBase::PlannerParameters::endElement(void *ctx, const char *name)
{
    if( _pcurreader != NULL ) {
        if( _pcurreader->endElement(ctx, name) ) {
            delete _pcurreader; _pcurreader = NULL;
        }
    }
    else if( stricmp((const char*)name, "initialconfig") == 0 ) {
        vinitialconfig.resize(0);
        dReal f;
        while( !_ss.eof() ) {
            _ss >> f;
            if( !_ss )
                break;
            vinitialconfig.push_back(f);
        }
    }
    else if( stricmp((const char*)name, "goalconfig") == 0 ) {
        vgoalconfig.resize(0);
        dReal f;
        while( !_ss.eof() ) {
            _ss >> f;
            if( !_ss )
                break;
            vgoalconfig.push_back(f);
        }
    }
    else if( stricmp((const char*)name, "parameters") == 0 ) {
        vParameters.resize(0);
        dReal f;
        while( !_ss.eof() ) {
            _ss >> f;
            if( !_ss )
                break;
            vParameters.push_back(f);
        }
    }
    else if( stricmp((const char*)name, "intparameters") == 0 ) {
        vnParameters.resize(0);
        int n;
        while( !_ss.eof() ) {
            _ss >> n;
            if( !_ss )
                break;
            vnParameters.push_back(n);
        }
    }
    else if( stricmp((const char*)name, "workspacegoal") == 0 ) {
        _ss >> tWorkspaceGoal;
        bHasWorkspaceGoal = true;
    }
    else if( stricmp((const char*)name, "maxiterations") == 0 ) {
        _ss >> nMaxIterations;
    }
    else {
        _pcurreader = new DummyXMLReader(name,"plannerparameters");
    }

    return false;
}

void PlannerBase::PlannerParameters::characters(void *ctx, const char *ch, int len)
{
    if( len > 0 ) {
        _ss.clear();
        _ss.str(string(ch, len));
    }
    else
        _ss.str(""); // reset
}

std::ostream& operator<<(std::ostream& O, const PlannerBase::PlannerParameters& v)
{
    O << "<PlannerParameters>" << endl;
    v.serialize(O);
    O << "</PlannerParameters>" << endl;
    return O;
}

#ifdef _WIN32
char *strcasestr(const char *s, const char *find)
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
    ((XMLREADERDATA*)ctx)->_preader->startElement(((XMLREADERDATA*)ctx)->_ctxt, (const char*)name, (const char**)atts);
}

void DefaultEndElementSAXFunc(void *ctx, const xmlChar *name)
{
    XMLREADERDATA* data = (XMLREADERDATA*)ctx;

    if( data->_preader->endElement(data->_ctxt, (const char*)name) ) {
        //RAVEPRINT(L"%s size read %d\n", name, data->_ctxt->input->consumed);
        xmlStopParser(data->_ctxt);
    }
}

void DefaultCharactersSAXFunc(void *ctx, const xmlChar *ch, int len)
{
    ((XMLREADERDATA*)ctx)->_preader->characters(((XMLREADERDATA*)ctx)->_ctxt, (const char*)ch, len);
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
    if (ctxt == NULL) return -1;
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

        char* p = strcasestr(buf.str().c_str(), "</PlannerParameters>");

        if( p != NULL ) {
            I.clear();
            I.seekg((size_t)pos+((p-buf.str().c_str())+20));
        }
        else
            RAVELOG_ERRORA("error, failed to find </PlannerParameters> in %s\n", buf.str().c_str());

        OneTagReader tagreader("PlannerParameters", &pp);
        LocalXML::ParseXMLData(&tagreader, buf.str().c_str(), -1);
    }

    return I;
}

// CmdProblemInstance class
CmdProblemInstance::~CmdProblemInstance()
{
    Destroy();
}

void CmdProblemInstance::Destroy()
{
    _mapCommands.clear();
}

bool CmdProblemInstance::SendCommand(const char* pcmd, std::string& response)
{
    if( pcmd == NULL )
        return false;
    
    std::stringstream sresponse;
    std::string cmd;
    
    // windows builds complain because of multiply defined symbosl
    std::stringstream sinput(pcmd);
    sinput >> cmd;

    if( !sinput )
        return false;
    
    CMDMAP::iterator it = _mapCommands.find(cmd);
    if( it == _mapCommands.end() ) {
        RAVELOG_WARNA("failed to find command %s in problem %s\n", cmd.c_str(), GetXMLId());
        return false;
    }
    
    if( !(this->*it->second.fn)(sresponse, sinput) ) {
        RAVELOG_WARNA("command error in problem %s: %s\n", GetXMLId(), pcmd);
        return false;
    }
    
    response = sresponse.str();
    return true;
}

bool CmdProblemInstance::RegisterCommand(const std::string& cmdname, CommandFn fncmd, const std::string& strhelp)
{
    if( fncmd == NULL || cmdname.size() == 0 || _mapCommands.find(cmdname) != _mapCommands.end() )
        return false;
    _mapCommands[cmdname] = COMMAND(fncmd, strhelp);
    return true;
}

bool CmdProblemInstance::RegisterCommand(const char* pcmdname, CommandFn fncmd, const char* pstrhelp)
{
    if( fncmd == NULL || pcmdname == NULL || _mapCommands.find(pcmdname) != _mapCommands.end() )
        return false;
    _mapCommands[pcmdname] = COMMAND(fncmd, pstrhelp != NULL ? std::string(pstrhelp) : std::string());
    return true;
}

bool CmdProblemInstance::DeleteCommand(const std::string& cmdname)
{
    CMDMAP::iterator it = _mapCommands.find(cmdname);
    if( it == _mapCommands.end() )
        return false;
    _mapCommands.erase(it);
    return true;
}

CmdProblemInstance::CommandFn CmdProblemInstance::GetCommand(const std::string& cmdname)
{
    CMDMAP::iterator it = _mapCommands.find(cmdname);
    if( it == _mapCommands.end() )
        return NULL;
    return it->second.fn;
}

const CmdProblemInstance::CMDMAP& CmdProblemInstance::GetCommands() const
{
    return _mapCommands;
}

void CmdProblemInstance::GetCommandHelp(std::ostream& o) const
{
    int maxlen = 0;
    CMDMAP::const_iterator it;
    for(it = _mapCommands.begin(); it != _mapCommands.end(); ++it) {
        if( maxlen  < (int)it->first.size() )
            maxlen = (int)it->first.size();
    }
    
    for(it = _mapCommands.begin(); it != _mapCommands.end(); ++it) {
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

void SensorBase::SetName(const wchar_t* pNewName)
{
    if( pNewName == NULL ) {
        _name.clear();
        return;
    }
    _name = pNewName;
}

void SensorBase::SetName(const char* pNewName)
{
    if( pNewName == NULL ) {
        _name.clear();
        return;
    }

    _name = _ravembstowcs(pNewName);
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

} // end namespace OpenRAVE
