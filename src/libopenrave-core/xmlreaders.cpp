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
#include "ravep.h"

/////////////////
// XML Parsing //
/////////////////
// define LIBXML_STATIC for static linking (libxml2s.lib)
#define LIBXML_SAX1_ENABLED
#include <libxml/globals.h>
#include <libxml/xmlerror.h>
#include <libxml/parser.h>
#include <libxml/parserInternals.h> // only for xmlNewInputFromFile()
#include <libxml/tree.h>

#include <libxml/debugXML.h>
#include <libxml/xmlmemory.h>

#include <iostream>
#include <sstream>

C_ASSERT(sizeof(xmlChar) == 1);

// Definitions from openravep
TemplateStreamParser<int, char> g_intparser;
TemplateStreamParser<float, char> g_floatparser;
TemplateStreamParser<dReal, char> g_realparser;

StringStreamParser g_stringparser;
WStringStreamParser g_wstringparser;

// the directory of the file currently parsing
static string s_strParseDirectory;
static string s_strFullFilename;

int g_XMLErrorCount = 0;

namespace OpenRAVE {
    int GetXMLErrorCount()
    {
        int ret = g_XMLErrorCount;
        g_XMLErrorCount = 0;
        return ret;
    }

    EnvironmentBase* CreateEnvironment(bool bLoadAllPlugins) { return new Environment(bLoadAllPlugins); }
    RaveServerBase* CreateSimpleTextServer(EnvironmentBase* penv) { return new RaveServer(penv); }
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

bool RaveParseXMLFile(Environment* penv, BaseXMLReader* preader, const wchar_t* filename)
{
    string strfile = _stdwcstombs(filename);
    return RaveParseXMLFile(penv, preader, strfile.c_str());
}

void RaveXMLErrorFunc(void *ctx, const char *msg, ...)
{
    va_list args;

     va_start(args, msg);
    RAVEPRINT(L"XML Parse error: ");
    wchar_t wfmt[200];
    swprintf(wfmt,200,L"%s",msg);
    vwprintf(wfmt,args);
    va_end(args);
    g_XMLErrorCount++;
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
#endif // LIBXML_SAX1_ENABLED

    ctxt->str_xml = xmlDictLookup(ctxt->dict, BAD_CAST "xml", 3);
    ctxt->str_xmlns = xmlDictLookup(ctxt->dict, BAD_CAST "xmlns", 5);
    ctxt->str_xml_ns = xmlDictLookup(ctxt->dict, XML_XML_NAMESPACE, 36);
    if ((ctxt->str_xml==NULL) || (ctxt->str_xmlns==NULL) || (ctxt->str_xml_ns == NULL))
        return false;
    return true;
}

int raveXmlSAXUserParseFile(xmlSAXHandlerPtr sax, BaseXMLReader* preader, const char *filename)
{
    int ret = 0;
    xmlParserCtxtPtr ctxt;
    
    ctxt = xmlCreateFileParserCtxt(filename);
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
    
    return ret;
}

int raveXmlSAXUserParseMemory(xmlSAXHandlerPtr sax, BaseXMLReader* preader, const char *buffer, int size)
{
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
    
    return ret;
}

static xmlSAXHandler s_DefaultSAXHandler = {0};

bool RaveParseXMLFile(Environment* penv, BaseXMLReader* preader, const char* filename)
{
    if( preader == NULL || filename == NULL )
        return false;

    if( !s_DefaultSAXHandler.initialized ) {
        // first time, so init
        s_DefaultSAXHandler.startElement = DefaultStartElementSAXFunc;
        s_DefaultSAXHandler.endElement = DefaultEndElementSAXFunc;
        s_DefaultSAXHandler.characters = DefaultCharactersSAXFunc;
        s_DefaultSAXHandler.error = RaveXMLErrorFunc;
        s_DefaultSAXHandler.initialized = 1;
    }

    // init the dir (for some reason msvc confuses the linking of the string class with soqt, so do it the old fashioned way)
    string olddir = s_strParseDirectory;
    string oldfile = s_strFullFilename;
    string appended;
    const char* p = strrchr(filename, '/');
    if( p == NULL ) p = strrchr(filename, '\\');

    // check for absolute paths
    if( filename[0] == '/' || filename[0] == '~' ) {
        s_strParseDirectory = "";
        s_strFullFilename = filename;
    }
    else {
        s_strFullFilename.resize(s_strParseDirectory.size()+strlen(filename));
        sprintf(&s_strFullFilename[0], "%s%s", s_strParseDirectory.c_str(), filename);
    }

    if( p != NULL ) {
        string temp = filename;
        temp.resize(p-filename+1);
        appended = temp;
        s_strParseDirectory += temp.c_str();
    }

#ifdef _WIN32
    const char filesep = '\\';
#else
    const char filesep = '/';
#endif

    // test if exists
    FILE* ftest = NULL;

    do {
        ftest = fopen(s_strFullFilename.c_str(), "r");
        if( ftest != NULL )
            break;
        
//        if( strstr(s_strFullFilename.c_str(), ".xml") == NULL ) {
//            s_strFullFilename += ".xml"; // append and try again
//            ftest = fopen(s_strFullFilename.c_str(), "r");
//            if( ftest != NULL )
//                break;
//        }
        
        // try the set openrave directories
        FOREACHC(itdir, penv->GetDataDirs()) {
            string newparse;
            newparse = *itdir; newparse.push_back(filesep);
            newparse += s_strParseDirectory;
            s_strFullFilename = newparse; s_strFullFilename += filename;
            //RAVEPRINT(L"trying %s\n", s_strFullFilename.c_str());
            
            ftest = fopen(s_strFullFilename.c_str(), "r");
            if( ftest != NULL ) {
                s_strParseDirectory = newparse; s_strParseDirectory += appended;
                break;
            }
            
            newparse = *itdir; newparse.push_back(filesep);
            s_strFullFilename = newparse; s_strFullFilename += filename;
            //RAVEPRINT(L"trying %s\n", s_strFullFilename.c_str());

            ftest = fopen(s_strFullFilename.c_str(), "r");
            if( ftest != NULL ) {
                s_strParseDirectory = newparse; s_strParseDirectory += appended;
                break;
            }
        }

        if( ftest != NULL )
            break;

    } while(0);
    
    if( ftest == NULL ) {
        g_XMLErrorCount++;
        s_strParseDirectory = olddir;
        s_strFullFilename = oldfile;
        return false;
    }

    fclose(ftest);
    //RAVEPRINT(L"fullfile: %s\n", s_strFullFilename.c_str());
    
    preader->_filename = s_strFullFilename;

    int ret = raveXmlSAXUserParseFile(&s_DefaultSAXHandler, preader, s_strFullFilename.c_str());
    if( ret != 0 ) {
        RAVEPRINT(L"xmlSAXUserParseFile: error parsing %s (error %d)\n", s_strFullFilename.c_str(), ret);
    }

    // restore
    s_strParseDirectory = olddir;
    s_strFullFilename = oldfile;

    // hmm....... necessary?
    //xmlCleanupParser();
    //xmlMemoryDump();

    return ret == 0;
}

bool RaveParseXMLData(Environment* penv, BaseXMLReader* preader, const char* pdata, int len)
{
    if( len <= 0 )
        len = strlen(pdata);
    if( !s_DefaultSAXHandler.initialized ) {
        // first time, so init
        s_DefaultSAXHandler.startElement = DefaultStartElementSAXFunc;
        s_DefaultSAXHandler.endElement = DefaultEndElementSAXFunc;
        s_DefaultSAXHandler.characters = DefaultCharactersSAXFunc;
        s_DefaultSAXHandler.error = RaveXMLErrorFunc;
        s_DefaultSAXHandler.initialized = 1;
    }

    return raveXmlSAXUserParseMemory(&s_DefaultSAXHandler, preader, pdata, len)==0;
}

map<PluginType, string> InitInterfaceNames()
{
    map<PluginType, string> m;
    m[PT_Planner] = "Planner";
    m[PT_Robot] = "Robot";
    m[PT_SensorSystem] = "SensorSystem";
    m[PT_Controller] = "Controller";
    m[PT_ProblemInstance] = "ProblemInstance";
    m[PT_InverseKinematicsSolver] = "InverseKinematicsSolver";
    m[PT_KinBody] = "KinBody";
    m[PT_PhysicsEngine] = "PhysicsEngine";
    m[PT_Sensor] = "Sensor";
    m[PT_CollisionChecker] = "CollisionChecker";
    m[PT_Trajectory] = "Trajectory";
    m[PT_Viewer] = "Viewer";
    m[PT_Server] = "Server";
    return m;
}

InterfaceXMLReader* CreateInterfaceReader(Environment* penv, PluginType type, InterfaceBase* pinterface, const string& xmltag, const char** atts)
{
    switch(type) {
    case PT_Planner: return new DummyInterfaceXMLReader<PT_Planner>(penv,pinterface,xmltag,atts);
    case PT_Robot: return new RobotXMLReader(penv,(RobotBase*)pinterface,atts);
    case PT_SensorSystem: return new DummyInterfaceXMLReader<PT_SensorSystem>(penv,pinterface,xmltag,atts);
    case PT_Controller: return new ControllerXMLReader(penv,pinterface,atts);
    case PT_ProblemInstance: return new DummyInterfaceXMLReader<PT_ProblemInstance>(penv,pinterface,xmltag,atts);
    case PT_InverseKinematicsSolver: return new DummyInterfaceXMLReader<PT_InverseKinematicsSolver>(penv,pinterface,xmltag,atts);
    case PT_KinBody: return new KinBodyXMLReader(penv,type, (KinBody*)pinterface,atts);
    case PT_PhysicsEngine: return new DummyInterfaceXMLReader<PT_PhysicsEngine>(penv,pinterface,xmltag,atts);
    case PT_Sensor: return new DummyInterfaceXMLReader<PT_Sensor>(penv,pinterface,xmltag,atts);
    case PT_CollisionChecker: return new DummyInterfaceXMLReader<PT_CollisionChecker>(penv,pinterface,xmltag,atts);
    case PT_Trajectory: return new DummyInterfaceXMLReader<PT_Trajectory>(penv,pinterface,xmltag,atts);
    case PT_Viewer: return new DummyInterfaceXMLReader<PT_Viewer>(penv,pinterface,xmltag,atts);
    case PT_Server: return new DummyInterfaceXMLReader<PT_Server>(penv,pinterface,xmltag,atts);
    }
    return NULL;
}

map<PluginType, string> g_mapInterfaceNames = InitInterfaceNames();
static map<PluginType, map<string, EnvironmentBase::CreateXMLReaderFn> > s_mmReaders;
static pthread_mutex_t s_mutexxmlreaders = PTHREAD_MUTEX_INITIALIZER;

void RegisterXMLReader(PluginType type, const char* xmltag, EnvironmentBase::CreateXMLReaderFn pfn)
{
    if( pfn != NULL && xmltag != NULL ) {
        MutexLock m(&s_mutexxmlreaders);
        s_mmReaders[type][xmltag] = pfn;
    }
}

void UnregisterXMLReader(PluginType type, const char* xmltag)
{
    if( xmltag != NULL ) {
        MutexLock m(&s_mutexxmlreaders);
        s_mmReaders[type].erase(xmltag);
    }
}
  
MASS MASS::GetBoxMass(Vector extents, Vector pos, dReal totalmass)
{
    MASS m;
    m.fTotalMass = totalmass;
    m.t = TransformMatrix();
    m.t.m[0] = totalmass/(dReal)12.0 * (extents.y*extents.y + extents.z*extents.z);
    m.t.m[4*1+1]= totalmass/(dReal)12.0 * (extents.x*extents.x + extents.z*extents.z);
    m.t.m[4*2+2]= totalmass/(dReal)12.0 * (extents.x*extents.x + extents.y*extents.y);
    m.t.trans = pos;
    return m;
}

MASS MASS::GetBoxMassD(Vector extents, Vector pos, dReal density)
{
    return GetBoxMass(extents, pos, 8.0f*extents.x*extents.y*extents.z*density);
}

MASS MASS::GetSphericalMass(dReal radius, Vector pos, dReal totalmass)
{
    MASS m;
    m.fTotalMass = totalmass;
    m.t = TransformMatrix();
    m.t.m[0] = m.t.m[4*1+1] = m.t.m[4*2+2]= (dReal)0.4 * totalmass * radius*radius;
    m.t.trans = pos;
    return m;
}

MASS MASS::GetSphericalMassD(dReal radius, Vector pos, dReal density)
{
    return GetSphericalMass(radius, pos, (dReal)4.0/(dReal)3.0 * M_PI * radius * radius * radius * density);
}

MASS MASS::GetCylinderMass(dReal radius, dReal height, Vector pos, dReal totalmass)
{
    MASS m;
    m.fTotalMass = totalmass;
    m.t = TransformMatrix();
    dReal r2 = radius*radius;
    // axis pointed toward z
    m.t.m[0] = m.t.m[4*1+1] = totalmass*(dReal(0.25)*r2 + (dReal(1.0)/dReal(12.0))*height*height);
    m.t.m[4*2+2] = totalmass*dReal(0.5)*r2;
    return m;
}

MASS MASS::GetCylinderMassD(dReal radius, dReal height, Vector pos, dReal density)
{
    return GetCylinderMass(radius, height, pos, M_PI*radius*radius*height*density);
}

MASS MASS::operator+(const MASS& r) const
{
    MASS mnew;
    mnew.t.trans = (fTotalMass*t.trans+r.fTotalMass*r.t.trans)*((dReal)1.0/(fTotalMass+r.fTotalMass));
    mnew.fTotalMass = fTotalMass + r.fTotalMass;
    mnew.t.m[0] = t.m[0] + r.t.m[0]; mnew.t.m[1] = t.m[1] + r.t.m[1]; mnew.t.m[2] = t.m[2] + r.t.m[2];
    mnew.t.m[4] = t.m[4] + r.t.m[4]; mnew.t.m[5] = t.m[5] + r.t.m[5]; mnew.t.m[6] = t.m[6] + r.t.m[6];
    mnew.t.m[8] = t.m[8] + r.t.m[8]; mnew.t.m[9] = t.m[9] + r.t.m[9]; mnew.t.m[10] = t.m[10] + r.t.m[10];
    return mnew;
}

MASS& MASS::operator+=(const MASS& r)
{
    t.trans = (fTotalMass*t.trans+r.fTotalMass*r.t.trans)*((dReal)1.0/(fTotalMass+r.fTotalMass));
    fTotalMass += r.fTotalMass;
    t.m[0] += r.t.m[0]; t.m[1] += r.t.m[1]; t.m[2] += r.t.m[2];
    t.m[4] += r.t.m[4]; t.m[5] += r.t.m[5]; t.m[6] += r.t.m[6];
    t.m[8] += r.t.m[8]; t.m[9] += r.t.m[9]; t.m[10] += r.t.m[10];
    return *this;
}

MASS& MASS::transform(const TransformMatrix& trans)
{
    TransformMatrix tinvrot;
    tinvrot.m[0] = trans.m[0]; tinvrot.m[1] = trans.m[4*1+0]; tinvrot.m[2] = trans.m[4*2+0];
    tinvrot.m[4] = trans.m[1]; tinvrot.m[5] = trans.m[4*1+1]; tinvrot.m[6] = trans.m[4*2+1];
    tinvrot.m[8] = trans.m[2]; tinvrot.m[9] = trans.m[4*1+2]; tinvrot.m[10] = trans.m[4*2+2];
    t = trans * t * tinvrot; // rotate mass about rotation

    // translate the inertia tensor
    dReal x = trans.trans.x, y = trans.trans.y, z = trans.trans.z;
    dReal x2 = x*x, y2 = y*y, z2 = z*z;
    t.m[0] += fTotalMass * (y2+z2); t.m[1] -= fTotalMass * x*y; t.m[2] -= fTotalMass * x * z;
    t.m[4] -= fTotalMass * y * z; t.m[5] += fTotalMass * (x2+z2); t.m[6] -= fTotalMass * y * z;
    t.m[8] -= fTotalMass * z * x; t.m[9] -= fTotalMass * z * y; t.m[10] += fTotalMass * (x2+y2);

    // ensure perfect symmetry
    t.m[5] = t.m[1];
    t.m[8] = t.m[2];
    t.m[9] = t.m[6];
    return *this;
}

// KinBody::Link Reader //
LinkXMLReader::LinkXMLReader(KinBody::Link* plink, KinBody* pparent, const char **atts)
{
    assert( pparent != NULL );

    _offsetfrom = NULL;
    _pcurparser = NULL;
    _pparent = pparent;
    _masstype = MT_None;
    _fMassDensity = 1;
    _vMassExtents = Vector(1,1,1);
    _bProcessingMass = false;
    _fTotalMass = 1;
    _pgeomprop = NULL;
    pKinBodyReader = NULL;
    _massCustom = MASS::GetSphericalMass(1,Vector(0,0,0),1);
    _plink = NULL;

    wchar_t* pname = NULL;
    bool bStaticSet = false;
    bool bStatic = false;
    const char* filename = NULL;

    if( atts != NULL ) {
        for (int i = 0;(atts[i] != NULL);i+=2) {
            if( stricmp((const char*)atts[i], "name") == 0 ) {
                size_t len = mbstowcs(NULL, (const char*)atts[i+1], 0);
                pname = new wchar_t[len+1];
                mbstowcs(pname, (const char*)atts[i+1], len+1);

                // check if a link with a previous name exists, is so, use it instead
                _plink = pparent->GetLink(pname);
            }
            else if( stricmp((const char*)atts[i], "type") == 0 ) {
                bStaticSet = true;
                if( stricmp((const char*)atts[i+1], "static") == 0 ) {
                    bStatic = true;
                }
            }
            else if( stricmp((const char*)atts[i], "file") == 0 ) {
                filename = (const char*)atts[i+1];
            }
        }
    }

    // if not appending to a body and plink pointer valid, append to it instead
    if( _plink == NULL && plink != NULL && plink->GetParent() == pparent )
        _plink = plink;

    if( filename != NULL ) {
        LinkXMLReader reader(_plink, pparent, NULL);
        bool bSuccess = RaveParseXMLFile((Environment*)_pparent->GetEnv(), &reader, filename);
        if( bSuccess )
            _plink = (KinBody::Link*)reader.Release();
        else
            reader.Release();
    }
    
    if( _plink == NULL )
        _plink = new KinBody::Link(pparent);

    if( pname != NULL )
        _plink->name = pname;
    
    if( bStaticSet ) {
        _plink->bStatic = bStatic;
    }
}

void LinkXMLReader::SetMassType(MassType type, float fValue, const Vector& vMassExtents)
{
    _masstype = type;
    _fMassDensity = _fTotalMass = fValue;
    _vMassExtents = vMassExtents;
}

void LinkXMLReader::startElement(void *ctx ATTRIBUTE_UNUSED, const char *name, const char **atts)
{
    if( !!_pcurreader ) {
        _pcurreader->startElement(ctx, name, atts);
    }
    else if( _pgeomprop != NULL ) {
        if( stricmp((const char*)name, "translation") == 0 ||
            stricmp((const char*)name, "rotationmat") == 0 ||
            stricmp((const char*)name, "rotationaxis") == 0 ||
            stricmp((const char*)name, "quat") == 0 ) {
            // check if quaternion or rotation matrix
            _pcurparser = &g_realparser;
        }
        else if( stricmp((const char*)name, "diffuseColor") == 0 ||
                 stricmp((const char*)name, "ambientColor") == 0 ||
                 stricmp((const char*)name, "transparency") == 0 ) {
            _pcurparser = &g_floatparser;
        }
        else if( stricmp((const char*)name, "render") == 0 ) {
            // check attributes for format (default is vrml)
            _pcurparser = &g_stringparser;
        }
        else {
            // could be type specific features
            if(_pgeomprop->GetType() == KinBody::Link::GEOMPROPERTIES::GeomTrimesh ) {
                if( stricmp((const char*)name, "data") == 0 ) {
                    // check attributes for format (default is vrml)
                    _pcurparser = &g_stringparser;
                }
                else
                    _pcurparser = &g_realparser;
            }
            else
                _pcurparser = &g_realparser;
        }
    }
    else if( _bProcessingMass ) {
        _pcurparser = &g_realparser;
    }
    else if( stricmp((const char*)name, "body") == 0 ) {
        tOrigTrans = _plink->GetTransform();
        _plink->SetTransform(Transform());
        _pcurreader.reset(new LinkXMLReader(_plink, _pparent, atts));
    }
    else if( stricmp((const char*)name, "geom") == 0 ) {
        assert( _pgeomprop == NULL );
        const char* type = NULL;
        bool bDraw = true;
        if( atts != NULL ) {
            for (int i = 0;(atts[i] != NULL);i+=2) {
                if( stricmp((const char*)atts[i], "type") == 0 ) {
                    type = (const char*)atts[i+1];
                }
                else if( stricmp((const char*)atts[i], "render") == 0 ) {
                    // set draw to false only if atts[i]==false
                    bDraw = stricmp((const char*)atts[i+1], "false")!=0;
                }
            }
        }

        if( type == NULL ) {
            RAVEPRINT(L"no geometry type, defaulting to box\n");
            type = "box";
        }

        _plink->_listGeomProperties.push_back(KinBody::Link::GEOMPROPERTIES());
        _pgeomprop = &_plink->_listGeomProperties.back();
        if( stricmp(type, "box") == 0 )
            _pgeomprop->type = KinBody::Link::GEOMPROPERTIES::GeomBox;
        else if( stricmp(type, "sphere") == 0 )
            _pgeomprop->type = KinBody::Link::GEOMPROPERTIES::GeomSphere;
        else if( stricmp(type, "cylinder") == 0 )
            _pgeomprop->type = KinBody::Link::GEOMPROPERTIES::GeomCylinder;
        else if( stricmp(type, "trimesh") == 0 ) {
            _pgeomprop->type = KinBody::Link::GEOMPROPERTIES::GeomTrimesh;
        }
        else {
            RAVEPRINT(L"type %s not supported\n", type);
        }

        _pgeomprop->bDraw = bDraw;
    }
    else if( stricmp((const char*)name, "translation") == 0 ) {
        _pcurparser = &g_realparser;
    }
    else if( stricmp((const char*)name, "rotationmat") == 0 ) {
        // check if quaternion or rotation matrix
        _pcurparser = &g_realparser;
    }
    else if( stricmp((const char*)name, "rotationaxis") == 0 ) {
        // check if quaternion or rotation matrix
        _pcurparser = &g_realparser;
    }
    else if( stricmp((const char*)name, "quat") == 0 ) {
        // check if quaternion or rotation matrix
        _pcurparser = &g_realparser;
    }
    else if( stricmp((const char*)name, "offsetfrom") == 0 ) {
        // check if quaternion or rotation matrix
        _pcurparser = &g_wstringparser;
    }
    else if( stricmp((const char*)name, "mass") == 0 ) {
        // find the type of mass and create
        _masstype = MT_Sphere;
        if( atts != NULL ) {
            for (int i = 0;(atts[i] != NULL);i++) {
                if( stricmp((const char*)atts[i], "type") == 0 ) {
                    if( stricmp((const char*)atts[i+1], "mimicgeom") == 0 ) {
                        _masstype = MT_MimicGeom;
                    }
                    else if( stricmp((const char*)atts[i+1], "box") == 0 ) {
                        _masstype = MT_Box;
                    }
                    else if( stricmp((const char*)atts[i+1], "sphere") == 0 ) {
                        _masstype = MT_Sphere;
                    }
                    else if( stricmp((const char*)atts[i+1], "custom") == 0 ) {
                        _masstype = MT_Custom;
                    }

                    break;
                }
            }
        }

        _bProcessingMass = true;
    }
    else {
        // start a new field
        _pcurreader.reset(new DummyXMLReader(name,"body"));
    }
}

bool LinkXMLReader::endElement(void *ctx ATTRIBUTE_UNUSED, const char *name)
{
    dReal* pf = _pcurparser != NULL ? (dReal*)_pcurparser->GetData() : NULL;

    if( !!_pcurreader ) {
        if( _pcurreader->endElement(ctx, name) ) {
            if( stricmp((const char*)name, "body") == 0 ) {
                KinBody::Link* pnewlink = (KinBody::Link*)_pcurreader->Release();
                if( pnewlink != _plink ) {
                    RAVEPRINT(L"deleting old body %S:%S due to file linking inconsistencies\n", _plink->GetParent()->GetName(), _plink->GetName());
                    delete _plink;
                }
                _plink = pnewlink;

                // directly apply transform to all geomteries
                Transform tnew = _plink->GetTransform();
                FOREACH(itgeom, _plink->_listGeomProperties)
                    itgeom->_t = tnew * itgeom->_t;
                _plink->SetTransform(tOrigTrans);
            }

            _pcurreader.reset();
        }
    }
    else if( _pgeomprop != NULL ) {
        if( stricmp((const char*)name, "translation") == 0 ) {
            assert( _pcurparser->GetCount() == 3 );
            _pgeomprop->_t.trans = Vector(pf[0], pf[1], pf[2]);
        }
        else if( stricmp((const char*)name, "rotationmat") == 0 ) {
            // check if quaternion or rotation matrix
            assert( _pcurparser->GetCount() == 9 );
            TransformMatrix tnew;
            tnew.m[0] = pf[0]; tnew.m[1] = pf[1]; tnew.m[2] = pf[2]; tnew.m[3] = 0;
            tnew.m[4] = pf[3]; tnew.m[5] = pf[4]; tnew.m[6] = pf[5]; tnew.m[7] = 0;
            tnew.m[8] = pf[6]; tnew.m[9] = pf[7]; tnew.m[10] = pf[8]; tnew.m[11] = 0;
            _pgeomprop->_t.rot = (Transform(tnew)*_pgeomprop->_t).rot;
        }
        else if( stricmp((const char*)name, "rotationaxis") == 0 ) {
            // check if quaternion or rotation matrix
            assert( _pcurparser->GetCount() == 4 );
            Transform tnew;
            normalize3(pf, pf);
            tnew.rotfromaxisangle(Vector(pf[0], pf[1], pf[2]), pf[3] * PI / 180.0f);
            _pgeomprop->_t.rot = (tnew*_pgeomprop->_t).rot;
        }
        else if( stricmp((const char*)name, "quat") == 0 ) {
            // check if quaternion or rotation matrix
            assert( _pcurparser->GetCount() == 4 );
            Transform tnew;
            normalize4(tnew.rot, pf);
            _pgeomprop->_t.rot = (tnew*_pgeomprop->_t).rot;
        }
        else if( stricmp((const char*)name, "render") == 0 ) {
            // check attributes for format (default is vrml)
            stringstream ss((char*)_pcurparser->GetData());
            string renderfile;
            _pgeomprop->vRenderScale = Vector(1,1,1);
            ss >> renderfile;
            ss >> _pgeomprop->vRenderScale.x; _pgeomprop->vRenderScale.y = _pgeomprop->vRenderScale.z = _pgeomprop->vRenderScale.x;
            ss >> _pgeomprop->vRenderScale.y >> _pgeomprop->vRenderScale.z;
            
            _pgeomprop->renderfile = pKinBodyReader->GetModelsDir(renderfile.c_str());
        }
        else if( stricmp((const char*)name, "diffuseColor") == 0 ) {
            float* pcol = (float*)_pcurparser->GetData();
            _pgeomprop->diffuseColor = RaveVector<float>(pcol[0], pcol[1], pcol[2]);
        }
        else if( stricmp((const char*)name, "ambientColor") == 0 ) {
            float* pcol = (float*)_pcurparser->GetData();
            _pgeomprop->ambientColor = RaveVector<float>(pcol[0], pcol[1], pcol[2]);
        }
        else if( stricmp((const char*)name, "transparency") == 0 ) {
            _pgeomprop->ftransparency = *(float*)_pcurparser->GetData();
        }
        else if( stricmp((const char*)name, "geom") == 0 ) {

            if( _pgeomprop->type == KinBody::Link::GEOMPROPERTIES::GeomCylinder ) { // axis has to point on y
                // rotate on x axis by pi/2
                Transform trot;
                trot.rotfromaxisangle(Vector(1, 0, 0), PI/2);
                _pgeomprop->_t.rot = (_pgeomprop->_t*trot).rot;
            }

            // call before attaching the geom
            if( _pgeomprop->type != KinBody::Link::GEOMPROPERTIES::GeomTrimesh ) {
                _pgeomprop->InitCollisionMesh();
            }

            KinBody::Link::TRIMESH trimesh = _pgeomprop->GetCollisionMesh();
            trimesh.ApplyTransform(_pgeomprop->_t);
            _plink->collision.Append(trimesh);
            _pgeomprop = NULL;
        }
        else {
            // could be type specific features
            switch(_pgeomprop->GetType()) {
            case KinBody::Link::GEOMPROPERTIES::GeomSphere:
                if( stricmp((const char*)name, "radius") == 0 ) {
                    assert( _pcurparser->GetCount() == 1 );
                    _pgeomprop->vGeomData.x = pf[0];
                }
                break;
            case KinBody::Link::GEOMPROPERTIES::GeomBox:
                if( stricmp((const char*)name, "extents") == 0 ) {
                    assert( _pcurparser->GetCount() == 3 );
                    _pgeomprop->vGeomData = Vector(pf[0], pf[1], pf[2]);
                }
                break;
            case KinBody::Link::GEOMPROPERTIES::GeomCylinder:
                    if( stricmp((const char*)name, "radius") == 0 ) _pgeomprop->vGeomData.x = pf[0];
                    else if( stricmp((const char*)name, "height") == 0 ) _pgeomprop->vGeomData.y = pf[0];
                    break;
            case KinBody::Link::GEOMPROPERTIES::GeomTrimesh:
                if( stricmp((const char*)name, "data") == 0 ) {
                    stringstream ss((char*)_pcurparser->GetData());
                    
                    string orgrenderfile;
                    Vector vScale(1,1,1);
                    ss >> orgrenderfile;
                    ss >> vScale.x; vScale.y = vScale.z = vScale.x;
                    ss >> vScale.y >> vScale.z;
                    string renderfile = pKinBodyReader->GetModelsDir(orgrenderfile.c_str());

                    bool bSuccess = false;
                    if( renderfile.size() > 0 ) {
#ifdef OPENRAVE_COIN3D
                        SoInput mySceneInput;
                        if (!mySceneInput.openFile(renderfile.c_str())) {
                            RAVEPRINT(L"Failed to open %s for KinBody:TriMesh\n", renderfile.c_str());
                            g_XMLErrorCount++;
                        }
                        else {
                            // SoDB::readAll memory leaks!
                            SoSeparator* psep = SoDB::readAll(&mySceneInput);
                            CreateTriMeshData(psep, _pgeomprop->collisionmesh);
                            psep->unref();

                            //_trimeshGeom
                            FOREACH(it, _pgeomprop->collisionmesh.vertices)
                                *it *= vScale;

                            bSuccess = true;
                        }
                        
                        mySceneInput.closeFile();
#endif

                        if( !bSuccess ) {
                            ivcon::ReadFile(renderfile.c_str(), _pgeomprop->collisionmesh);
                            RAVELOG_VERBOSEA("trimesh verts: %"PRIdS", inds: %"PRIdS"\n", _pgeomprop->collisionmesh.vertices.size(), _pgeomprop->collisionmesh.indices.size());
                            FOREACH(it, _pgeomprop->collisionmesh.vertices)
                                *it *= vScale;
                        }
                    }
                    else
                        RAVELOG_WARNA("failed to find %s\n", orgrenderfile.c_str());
                }
                break;
            default:
                assert(0);
            }
        }
    }
    else if( stricmp((const char*)name, "body") == 0 ) {

        if( _plink->GetGeometries().size() == 0 )
            RAVEPRINT(L"link %S has no geometry attached!\n", _plink->GetName());
        assert(_plink->GetGeometries().size() > 0 );

        // perform final processing stages
        MASS totalmass;
        if( _masstype == MT_MimicGeom ) {
            FOREACHC(itgeom, _plink->GetGeometries()) {
                MASS mass;
                switch(itgeom->type) {
                case KinBody::Link::GEOMPROPERTIES::GeomSphere:
                    mass = MASS::GetSphericalMassD(itgeom->GetSphereRadius(), Vector(),_fMassDensity);
                case KinBody::Link::GEOMPROPERTIES::GeomBox:
                    mass = MASS::GetBoxMassD(itgeom->GetBoxExtents(), Vector(), _fMassDensity);
                case KinBody::Link::GEOMPROPERTIES::GeomCylinder:
                    mass = MASS::GetCylinderMassD(itgeom->GetCylinderRadius(), itgeom->GetCylinderHeight(), Vector(), _fMassDensity);
                default:
                    break;
                }
                    
                totalmass += mass.transform(itgeom->GetTransform());
            }
        }
        else if( _masstype == MT_Box )
            totalmass = MASS::GetBoxMassD(_vMassExtents, Vector(), _fMassDensity);
        else if( _masstype == MT_BoxMass )
            totalmass = MASS::GetBoxMass(_vMassExtents, Vector(), _fTotalMass);
        else if( _masstype == MT_Sphere )
            totalmass = MASS::GetSphericalMassD(_vMassExtents.x, Vector(), _fMassDensity);
        else if( _masstype == MT_Custom )
            totalmass = _massCustom;
        else
            totalmass = MASS::GetSphericalMassD(_vMassExtents.x, Vector(), _fTotalMass);
        
        _plink->_transMass = totalmass.t;
        _plink->_mass = totalmass.fTotalMass;
        tOrigTrans = _plink->GetTransform();

        Transform cur;
            
        if( _offsetfrom != NULL ) {
            // recompute new transformation
            Transform root;
            if( pKinBodyReader != NULL )
                root = pKinBodyReader->GetOffsetFrom(_offsetfrom);
            else
                root = _offsetfrom->GetTransform();

            cur = _plink->GetTransform();
            cur = root * cur;
            tOrigTrans = root * tOrigTrans; // update orig trans separately
            _plink->SetTransform(cur);
        }

        return true;
    }
    else if( _bProcessingMass ) {
        if( stricmp((const char*)name, "density") == 0 ) {
            if( _masstype == MT_BoxMass ) _masstype = MT_Box;
            else if( _masstype == MT_SphereMass) _masstype = MT_Sphere;
            _fMassDensity = pf[0];
        }
        else if( stricmp((const char*)name, "total") == 0 ) {
            if( _masstype == MT_Box ) _masstype = MT_BoxMass;
            else if( _masstype == MT_Sphere) _masstype = MT_SphereMass;
            _fTotalMass = _massCustom.fTotalMass = pf[0];
        }
        else if( stricmp((const char*)name, "radius") == 0 ) {
            _vMassExtents.x = pf[0];
        }
        else if( _masstype == MT_Box && stricmp((const char*)name, "extents") == 0 ) {
            assert( _pcurparser->GetCount() == 3 );
            _vMassExtents = Vector(pf[0], pf[1], pf[2]);
        }
        else if( _masstype == MT_Custom ) {
            if( stricmp((const char*)name, "com") == 0 ) {
                if( _pcurparser->GetCount() == 3  ) {
                    _plink->_transMass.trans = Vector(pf[0],pf[1],pf[2]);
                    _massCustom.t.trans = Vector(pf[0],pf[1],pf[2]);
                }
                else
                    RAVELOG(L"bad center of mass\n");
            }
            else if( stricmp((const char*)name, "inertia") == 0 ) {
                if( _pcurparser->GetCount() == 9  ) {
                    _massCustom.t.m[0] = pf[0]; _massCustom.t.m[1] = pf[1]; _massCustom.t.m[2] = pf[2];
                    _massCustom.t.m[4] = pf[3]; _massCustom.t.m[5] = pf[4]; _massCustom.t.m[6] = pf[5];
                    _massCustom.t.m[8] = pf[6]; _massCustom.t.m[9] = pf[7]; _massCustom.t.m[10] = pf[8];
                }
                else
                    RAVELOG(L"bad rotational inertia\n");
            }
        }
        
        if( stricmp((const char*)name, "mass") == 0 ) {
            assert( _bProcessingMass );
            _bProcessingMass = false;
        }
    }
    else if( stricmp((const char*)name, "translation") == 0 ) {
        assert( _pcurparser->GetCount() == 3 );
        _plink->_t.trans += Vector(pf[0], pf[1], pf[2]);
    }
    else if( stricmp((const char*)name, "rotationmat") == 0 ) {
        // check if quaternion or rotation matrix
        assert( _pcurparser->GetCount() == 9 );
        TransformMatrix tnew;
        tnew.m[0] = pf[0]; tnew.m[1] = pf[1]; tnew.m[2] = pf[2]; tnew.m[3] = 0;
        tnew.m[4] = pf[3]; tnew.m[5] = pf[4]; tnew.m[6] = pf[5]; tnew.m[7] = 0;
        tnew.m[8] = pf[6]; tnew.m[9] = pf[7]; tnew.m[10] = pf[8]; tnew.m[11] = 0;
        _plink->_t.rot = (Transform(tnew)*_plink->_t).rot;
    }
    else if( stricmp((const char*)name, "rotationaxis") == 0 ) {
        // check if quaternion or rotation matrix
        assert( _pcurparser->GetCount() == 4 );
        
        Transform tnew;
        normalize3(pf, pf);
        tnew.rotfromaxisangle(Vector(pf[0], pf[1], pf[2]), pf[3]*PI/180.0f);
        _plink->_t.rot = (tnew*_plink->_t).rot;
    }
    else if( stricmp((const char*)name, "quat") == 0 ) {
        // check if quaternion or rotation matrix
        assert( _pcurparser->GetCount() == 4 );
        
        Transform tnew;
        normalize4(tnew.rot, pf);
        _plink->_t.rot = (tnew*_plink->_t).rot;
    }
    else if( stricmp((const char*)name, "offsetfrom") == 0 ) {
        // figure out which body
        KinBody::Link* plink = _pparent->GetLink((const wchar_t*)_pcurparser->GetData());

        if( plink == NULL ) {
            RAVEPRINT(L"Failed to find offsetfrom body %ls\n", (const wchar_t*)_pcurparser->GetData());
            g_XMLErrorCount++;
        }
        else _offsetfrom = plink;
    }

    _pcurparser = NULL;
    return false;
}

void LinkXMLReader::characters(void *ctx ATTRIBUTE_UNUSED, const char *ch, int len)
{
    if( !!_pcurreader )
        _pcurreader->characters(ctx, ch, len);
    else if( _pcurparser != NULL )
        _pcurparser->Format((const char*)ch, len);
}

// KinBody::Joint Reader //
JointXMLReader::JointXMLReader(KinBody* pparent, const char **atts)
{
    assert( atts != NULL && pparent != NULL );

    _bMimicJoint = false;
    bDisabled = false;
    _pparent = pparent;
    _pjoint = new KinBody::Joint(pparent);
    _pcurparser = NULL;
    _offsetfrom = NULL;
    pKinBodyReader = NULL;
    fWeights[0] = fWeights[1] = fWeights[2] = 1;
    _pjoint->type = KinBody::Joint::JointHinge;

    if( atts != NULL ) {
        for (int i = 0;(atts[i] != NULL);i+=2) {
            if( stricmp((const char*)atts[i], "name") == 0 ) {
                _pjoint->name = _ravembstowcs((const char*)atts[i+1]);
            }
            else if( stricmp((const char*)atts[i], "type") == 0 ) {
                if( stricmp((const char*)atts[i+1], "hinge") == 0 )
                    _pjoint->type = KinBody::Joint::JointHinge;
                else if( stricmp((const char*)atts[i+1], "slider") == 0 )
                    _pjoint->type = KinBody::Joint::JointSlider;
                else if( stricmp((const char*)atts[i+1], "universal") == 0 )
                    _pjoint->type = KinBody::Joint::JointUniversal;
                else if( stricmp((const char*)atts[i+1], "hinge2") == 0 )
                    _pjoint->type = KinBody::Joint::JointHinge2;
                else {
                    RAVEPRINT(L"unrecognized joint type: %s, setting to hinge\n", atts[i+1]);
                    _pjoint->type = KinBody::Joint::JointHinge;
                }
            }
            else if( stricmp((const char*)atts[i], "enable") == 0 ) {
                bDisabled = stricmp((const char*)atts[i+1], "false") == 0;
            }
            else if( stricmp((const char*)atts[i], "mimic") == 0 ) {
                // format: jointname coeff0 coeff1
                
                // find the joint
                char jointname[64];
                float f[2] = {1,0};

                sscanf((const char*)atts[i+1], "%s %f %f", jointname, f, f+1);
                
                _pjoint->fMimicCoeffs[0] = f[0];
                _pjoint->fMimicCoeffs[1] = f[1];
                _bMimicJoint = true;
                _strmimicjoint = jointname;
            }
        }
    }

    _pjoint->_vlowerlimit.resize(_pjoint->GetDOF());
    _pjoint->_vupperlimit.resize(_pjoint->GetDOF());
    for(int i = 0; i < _pjoint->GetDOF(); ++i) {
        _pjoint->_vlowerlimit[i] = -PI;
        _pjoint->_vupperlimit[i] = PI;
    }
}

JointXMLReader::~JointXMLReader()
{
    assert(_pjoint==NULL);
}

void JointXMLReader::startElement(void *ctx ATTRIBUTE_UNUSED, const char *name, const char **atts)
{
    if( !!_pcurreader ) {
        _pcurreader->startElement(ctx, name, atts);
    }
    else if( stricmp((const char*)name, "body") == 0 || stricmp((const char*)name, "offsetfrom") == 0) {
        _pcurparser = &g_wstringparser;
    }
    else if( stricmp((const char*)name, "weight") == 0 ||
             stricmp((const char*)name, "lostop") == 0 ||
             stricmp((const char*)name, "histop") == 0 ||
             stricmp((const char*)name, "maxvel") == 0 ||
             stricmp((const char*)name, "maxaccel") == 0 ||
             stricmp((const char*)name, "maxtorque") == 0 ||
             stricmp((const char*)name, "maxforce") == 0 ||
             stricmp((const char*)name, "resolution") == 0 ||
             stricmp((const char*)name, "anchor") == 0 ||
             stricmp((const char*)name, "axis") == 0 ||
             stricmp((const char*)name, "axis1") == 0 ||
             stricmp((const char*)name, "axis2") == 0 ||
             stricmp((const char*)name, "axis3") == 0 ||
             stricmp((const char*)name, "mode") == 0 ) {
        _pcurparser = &g_realparser;
    }
    else {
        // start a new field
        _pcurreader.reset(new DummyXMLReader(name,"joint"));
    }
}

bool JointXMLReader::endElement(void *ctx ATTRIBUTE_UNUSED, const char *name)
{
    const dReal* pf = _pcurparser != NULL ? (const dReal*)_pcurparser->GetData() : NULL;
    int numindices = _pjoint->GetDOF();
    dReal fRatio = _pjoint->type == KinBody::Joint::JointSlider ? (dReal)1 : (dReal)PI / 180.0f; // most, but not all, joint take degrees

    if( !!_pcurreader ) {
        if( _pcurreader->endElement(ctx, name) ) {
            _pcurreader.reset();
        }
    }
    else if( stricmp((const char*)name, "joint") == 0 ) {

        if( _pparent->GetLinks().size() == 0 ) {
            RAVEPRINT(L"parent kinbody has no links!\n");
            return false;
        }

        wstring defaultname = L"J_";

        int numbad = 0;
        _pjoint->offset = 0;

        // check if joint needs an artificial offset, only for revolute joints that have identifying points!
        if( _pjoint->type == KinBody::Joint::JointHinge ||
            _pjoint->type == KinBody::Joint::JointUniversal ||
            _pjoint->type == KinBody::Joint::JointHinge2 ) {
            
            for(int i = 0; i < numindices; ++i) {
                if( _pjoint->_vlowerlimit[i] < -PI || _pjoint->_vupperlimit[i] > PI ) {
                    _pjoint->offset += 0.5f * (_pjoint->_vlowerlimit[i] + _pjoint->_vupperlimit[i]);
                    ++numbad;
                }
            }
            
            if( numbad > 0 ) {
                _pjoint->offset *= 1.0f / (float)numbad;
            }
        }

        Transform tbody0, tbody1;

        if( _pjoint->bodies[0] == NULL || _pjoint->bodies[1] == NULL ) {
            RAVEPRINT(L"one or more attached bodies are invalid for joint %S\n", _pjoint->GetName());
            if( _pjoint->bodies[1] == NULL )
                _pjoint->bodies[1] = _pparent->GetLinks().front();
            if( _pjoint->bodies[0] == NULL )
                _pjoint->bodies[0] = _pparent->GetLinks().front();
        }

        // make sure first body is always closer to the root, unless the second body is static
        if( !_pjoint->bodies[1]->IsStatic() ) {
            if( _pjoint->bodies[0]->IsStatic() || (_pjoint->bodies[0]->GetIndex() > _pjoint->bodies[1]->GetIndex() && !_pjoint->bodies[1]->IsStatic()) ) {
                for(int i = 0; i < _pjoint->GetDOF(); ++i)
                    _pjoint->vAxes[i] = -_pjoint->vAxes[i];
                swap(_pjoint->bodies[0], _pjoint->bodies[1]);
            }
        }

        tbody0 = _pjoint->bodies[0]->GetTransform();
        tbody1 = _pjoint->bodies[1]->GetTransform();

        Transform toffsetfrom;
        if( _offsetfrom != NULL ) {
            if( pKinBodyReader != NULL )
                toffsetfrom = pKinBodyReader->GetOffsetFrom(_offsetfrom);
            else
                toffsetfrom = _offsetfrom->GetTransform();
        }

        Transform trel;
        if( _pjoint->bodies[1]->IsStatic() ) {
            trel = tbody1.inverse() * tbody0;
            toffsetfrom = tbody1.inverse() * toffsetfrom;
        }
        else {
            trel = tbody0.inverse() * tbody1;
            toffsetfrom = tbody0.inverse() * toffsetfrom;
        }

        if( _pjoint->bodies[0]->IsStatic() ) {
            RAVEPRINT(L"joint %S: all attached links are static!\n", _pjoint->GetName());
        }

        _pjoint->vanchor = toffsetfrom*_pjoint->vanchor;
        for(int i = 0; i < _pjoint->GetDOF(); ++i)
            _pjoint->vAxes[i] = toffsetfrom.rotate(_pjoint->vAxes[i]);

        switch(_pjoint->type) {
        case KinBody::Joint::JointHinge:
            _pjoint->tLeft.rotfromaxisangle(_pjoint->vAxes[0], -_pjoint->offset);
            _pjoint->tLeft.trans = _pjoint->vanchor;
            _pjoint->tRight.trans = -_pjoint->vanchor;
            _pjoint->tRight = _pjoint->tRight * trel;
            break;
        case KinBody::Joint::JointSlider:
            _pjoint->tRight = trel;
            break;
        case KinBody::Joint::JointUniversal:
            _pjoint->tLeft.trans = _pjoint->vanchor;
            _pjoint->tRight.trans = -_pjoint->vanchor;
            _pjoint->tRight = _pjoint->tRight * trel;
            break;
        case KinBody::Joint::JointHinge2:
            _pjoint->tLeft.trans = _pjoint->vanchor;
            _pjoint->tRight.trans = -_pjoint->vanchor;
            _pjoint->tRight = _pjoint->tRight * trel;
            break;
        default:
            RAVEPRINT(L"unknown joint type %d\n", _pjoint->type);
            _pjoint->tLeft.identity();
            _pjoint->tRight.identity();
            
            break;
        }

        if( _pjoint->bodies[1]->IsStatic() ) {
            _pjoint->tLeft = _pparent->GetTransform().inverse() * tbody1 * _pjoint->tLeft;
            _pjoint->bodies[0]->SetTransform(_pjoint->tLeft * _pjoint->tRight);
        }
        else
            _pjoint->bodies[1]->SetTransform(tbody0 * _pjoint->tLeft * _pjoint->tRight);

        _pjoint->tinvRight = _pjoint->tRight.inverse();
        _pjoint->tinvLeft = _pjoint->tLeft.inverse();

        for(int i = 0; i < _pjoint->GetDOF(); ++i)
            _pparent->_vecJointWeights.push_back(fWeights[i]);

        // have to transform back
        if( _pjoint->bodies[0] != NULL )
            _pjoint->bodies[0]->SetTransform(tbody0);
        if( _pjoint->bodies[1] != NULL )
            _pjoint->bodies[1]->SetTransform(tbody1);

        return true;
    }
    else if( stricmp((const char*)name, "weight") == 0 ) {
        for(int i = 0; i < min(numindices, _pcurparser->GetCount()); ++i)
            fWeights[i] = pf[i];
    }
    else if( stricmp((const char*)name, "body") == 0 ) {
        // figure out which body
        int index = _pjoint->bodies[0] != NULL ? 1 : 0;
        bool bQuery = true;

        vector<KinBody::Link*>::const_iterator it;
        FORIT(it, _pparent->GetLinks()) {
#ifdef __APPLE_CC__
	  char strname1[256], strname2[256];
	  sprintf(strname1, "%S", (*it)->GetName());
	  sprintf(strname2, "%S", _pcurparser->GetData());
	  if( stricmp(strname1, strname2) == 0 ) {
#else
            if( wcsicmp((*it)->GetName(), (const wchar_t*)_pcurparser->GetData()) == 0 ) {
#endif        
	      bQuery = !(*it)->IsStatic();
                _pjoint->bodies[index] = *it;
                
                break;
            }
        }

      if( _pjoint->bodies[index] == NULL && bQuery ) {
          RAVEPRINT(L"Failed to find body %ls for joint %ls\n", (const wchar_t*)_pcurparser->GetData(), _pjoint->name.c_str());
          g_XMLErrorCount++;
      }
    }
    else if( stricmp((const char*)name, "lostop") == 0 ) {
        _pjoint->_vlowerlimit.resize(min(numindices, _pcurparser->GetCount()));
        for(int i = 0; i < min(numindices, _pcurparser->GetCount()); ++i)
            _pjoint->_vlowerlimit[i] = pf[i]*fRatio;
    }
    else if( stricmp((const char*)name, "histop") == 0 ) {
        _pjoint->_vupperlimit.resize(min(numindices, _pcurparser->GetCount()));
        for(int i = 0; i < min(numindices, _pcurparser->GetCount()); ++i)
            _pjoint->_vupperlimit[i] = pf[i]*fRatio;
    }
    else if( stricmp((const char*)name, "maxvel") == 0 )
        _pjoint->fMaxVel = pf[0];
    else if( stricmp((const char*)name, "maxaccel") == 0 )
        _pjoint->fMaxAccel = pf[0];
    else if( stricmp((const char*)name, "maxtorque") == 0 )
        _pjoint->fMaxTorque = pf[0];
    else if( stricmp((const char*)name, "resolution") == 0 ) {
        _pjoint->fResolution = *(const dReal*)_pcurparser->GetData() *fRatio;
    }
    else if( stricmp((const char*)name, "offsetfrom") == 0 ) {
        // figure out which body
        KinBody::Link* plink = _pparent->GetLink((const wchar_t*)_pcurparser->GetData());
        
        if( plink == NULL ) {
            RAVEPRINT(L"Failed to find body %ls\n", _pcurparser->GetData());
            g_XMLErrorCount++;
        }
        else _offsetfrom = plink;
    }
    else {
        // could be type specific
        switch(_pjoint->type) {
        case KinBody::Joint::JointHinge:
            if( stricmp((const char*)name, "anchor") == 0 ) _pjoint->vanchor = Vector(pf[0], pf[1], pf[2]);
            else if( stricmp((const char*)name, "axis") == 0 ) _pjoint->vAxes[0] = Vector(pf[0], pf[1], pf[2]).normalize3();
            break;
        case KinBody::Joint::JointSlider:
            if( stricmp((const char*)name, "axis") == 0 ) _pjoint->vAxes[0] = Vector(pf[0], pf[1], pf[2]).normalize3();
            break;
        case KinBody::Joint::JointUniversal:
            if( stricmp((const char*)name, "anchor") == 0 ) _pjoint->vanchor = Vector(pf[0], pf[1], pf[2]);
            else if( stricmp((const char*)name, "axis1") == 0 ) _pjoint->vAxes[0] = Vector(pf[0], pf[1], pf[2]).normalize3();
            else if( stricmp((const char*)name, "axis2") == 0 ) _pjoint->vAxes[1] = Vector(pf[0], pf[1], pf[2]).normalize3();
            break;
        case KinBody::Joint::JointHinge2:
            if( stricmp((const char*)name, "anchor") == 0 ) _pjoint->vanchor = Vector(pf[0], pf[1], pf[2]);
            else if( stricmp((const char*)name, "axis1") == 0 ) _pjoint->vAxes[0] = Vector(pf[0], pf[1], pf[2]).normalize3();
            else if( stricmp((const char*)name, "axis2") == 0 ) _pjoint->vAxes[1] = Vector(pf[0], pf[1], pf[2]).normalize3();
            break;
        default:
            assert(0);
            break;
        }
    }

    _pcurparser = NULL;
    return false;
}

void JointXMLReader::characters(void *ctx ATTRIBUTE_UNUSED, const char *ch, int len)
{
    if( !!_pcurreader )
        _pcurreader->characters(ctx, ch, len);
    else if( _pcurparser != NULL )
        _pcurparser->Format((const char*)ch, len);
}

InterfaceXMLReader::InterfaceXMLReader(Environment* penv, PluginType type, InterfaceBase* pinterface, const string& xmltag, const char **atts) : _penv(penv), _type(type), _pinterface(pinterface), _xmltag(xmltag)
{
    const char* ptype = NULL;
    if( atts != NULL ) {
        for (int i = 0;(atts[i] != NULL);i+=2) {
            if( stricmp((const char*)atts[i], "type") == 0 ) {
                ptype = (const char*)atts[i+1];
            }
            else if( stricmp((const char*)atts[i], "file") == 0 ) {
                strcpy((char*)atts[i], ""); // delete file attr
                boost::shared_ptr<BaseXMLReader> preader(CreateInterfaceReader(_penv,_type,_pinterface, xmltag, atts));
                bool bSuccess = RaveParseXMLFile(_penv,preader.get(), (const char*)atts[i+1]);
                if( !bSuccess ) {
                    preader->Release();
                    RAVELOG(L"Failed to load kinbody filename %s\n", atts[i+1]);
                    g_XMLErrorCount++;
                    break;
                }
                
                _pinterface = (InterfaceBase*)preader->Release();
                //_pchain->strXMLFilename = reader._filename;
            }
        }
    }

    if( _pinterface == NULL ) {
        if( ptype != NULL ) {
            _pinterface = _penv->CreateInterface(type,ptype);
            if( _pinterface == NULL )
                g_XMLErrorCount++;
        }
        
        if( _pinterface == NULL ) {
            switch(type) {
            case PT_KinBody:
                _pinterface = penv->CreateKinBody();
                break;
            case PT_Robot:
                _pinterface = _penv->CreateInterface(PT_Robot, "GenericRobot");
                if( _pinterface == NULL )
                    _penv->CreateInterface(PT_Robot, "");
                break;
            case PT_Controller:
                assert(0);
                _pinterface = _penv->CreateInterface(PT_Controller, "IdealController");
                break;
            default:
                _pinterface = _penv->CreateInterface(type, "");
                break;
            }
        }
    }

    if( _pinterface == NULL )
        RAVELOG_ERRORA("xml readers failed to create instance of type %d\n",type);
    else
        _type = _pinterface->GetInterfaceType();

    if( _xmltag.size() == 0 )
        _xmltag = g_mapInterfaceNames[_type];
}

void InterfaceXMLReader::startElement(void *ctx ATTRIBUTE_UNUSED, const char *name, const char **atts)
{
    if( _pinterface == NULL )
        return;

    if( !!_pcustomreader ) {
        _pcustomreader->startElement(ctx, name, atts);
    }
    else {
        // check for registers readers
        map<string, EnvironmentBase::CreateXMLReaderFn>::iterator it = s_mmReaders[_type].find(name);
        if( it != s_mmReaders[_type].end() ) {
            _readername = name;
            _pcustomreader.reset(it->second(_pinterface, atts));
        }
    }
}

bool InterfaceXMLReader::endElement(void *ctx ATTRIBUTE_UNUSED, const char *name)
{
    if( !!_pcustomreader ) {
        if( _pcustomreader->endElement(ctx, name) ) {
            if( _readername.size() > 0 )
                _pinterface->__mapReadableInterfaces[_readername] = (XMLReadable*)_pcustomreader->Release();
            _pcustomreader.reset();
        }
    }
    else if( stricmp((const char*)name, _xmltag.c_str()) == 0 )
        return true;
    return false;
}

void InterfaceXMLReader::characters(void *ctx ATTRIBUTE_UNUSED, const char *ch, int len)
{
    if( !!_pcustomreader )
        _pcustomreader->characters(ctx, ch, len);
}

//// KinBody Reader ////
KinBodyXMLReader::KinBodyXMLReader(Environment* penv, PluginType type, KinBody* pchain, const char **atts) : InterfaceXMLReader(penv,type,pchain,"kinbody",atts)
{
    _penv = penv;
    _pcurparser = NULL;
    _pchain = (KinBody*)_pinterface;
    assert( _pchain != NULL );
    _masstype = LinkXMLReader::MT_None;
    _fMassValue = 1;
    _vMassExtents = Vector(1,1,1);
    _bProcessingMass = false;
    _bOverwriteDiffuse = false;
    _bOverwriteAmbient = false;
    _bOverwriteTransparency = false;

    if( pchain != NULL ) {
        pchain->GetBodyTransformations(_vTransforms);
        rootoffset = _vTransforms.size();
        rootjoffset = (int)pchain->GetJoints().size();
    }
    else {
        rootoffset = 0;
        rootjoffset = 0;
    }

    if( atts != NULL ) {
        for (int i = 0;(atts[i] != NULL);i+=2) {
            if( stricmp((const char*)atts[i], "prefix") == 0 ) {
                prefix = (const char*)atts[i+1];
                wprefix = _ravembstowcs((const char*)atts[i+1]);
            }
            else if( stricmp((const char*)atts[i], "name") == 0 ) {
                _bodyname = (const char*)atts[i+1];
            }
        }
    }

    // reisze _vTransforms to be the same size as the initial number of links
    _pchain->GetBodyTransformations(_vTransforms);
    _pchain->SetGuiData(NULL);
}

const Transform KinBodyXMLReader::GetOffsetFrom(KinBody::Link* plink)
{
    assert(plink != NULL);
    if( plink->GetIndex() < 0 || plink->GetIndex() >= (int)_vTransforms.size() )
        return plink->GetTransform();
    return _vTransforms[plink->GetIndex()];
}
 
string KinBodyXMLReader::GetModelsDir(const char* pfilename) const
{
    assert( pfilename != NULL );
#ifdef _WIN32
    if( strchr(pfilename,':') != NULL )
        return pfilename;
#else
    if( pfilename[0] == '/' || pfilename[0] == '~' )
        return string(pfilename);
#endif

#ifdef _WIN32
    const char filesep = '\\';
#else
    const char filesep = '/';
#endif

    list<string> listmodelsdir;
    listmodelsdir.push_back(_strModelsDir);
    if( s_strParseDirectory.size() > 0 ) {
        listmodelsdir.push_back(s_strParseDirectory);
        listmodelsdir.back().push_back(filesep);
        listmodelsdir.back() += _strModelsDir;
    }

    string temp;
    FOREACH(itmodelsdir, listmodelsdir) {
        temp = *itmodelsdir + pfilename;
        //RAVELOG_INFOA("modelsdir: %s, modelname: %s\n", itmodelsdir->c_str(), temp.c_str());
        FILE* ftest = fopen(temp.c_str(), "r");
        if( ftest != NULL ) {
            fclose(ftest);
            return temp;
        }

        FOREACHC(itdir, _penv->GetDataDirs()) {
            temp = *itdir; temp.push_back(filesep);
            temp += *itmodelsdir;
            temp += pfilename;
            //RAVELOG_INFOA("name: %s\n", temp.c_str());
            ftest = fopen(temp.c_str(), "r");
            if( ftest != NULL ) {
                fclose(ftest);
                return temp;
            }
        }
    }

//    if( strModelsDir.size() > 0 ) {
//        FOREACHC(itdir, g_Environ.GetDataDirs()) {
//            temp = *itdir; temp.push_back(filesep);
//            temp += pfilename;
//            //RAVEPRINT(L"name: %s\n", temp.c_str());
//            ftest = fopen(temp.c_str(), "r");
//            if( ftest != NULL ) {
//                fclose(ftest);
//                return temp;
//            }
//        }
//    }

    return ""; // bad filename
}

void KinBodyXMLReader::startElement(void *ctx ATTRIBUTE_UNUSED, const char *name, const char **atts)
{
    if( !!_pcurreader ) {
        _pcurreader->startElement(ctx, name, atts);
        return;
    }

    if( _bProcessingMass ) {
        _pcurparser = &g_realparser;
        return;
    }
    
    InterfaceXMLReader::startElement(ctx,name,atts);
    if( !!_pcustomreader )
        return;

    if( stricmp((const char*)name, "kinbody") == 0 ) {
        _pcurreader.reset(new KinBodyXMLReader(_penv,_pchain->GetInterfaceType(),_pchain, atts));
//        if( atts != NULL ) {
//            for (int i = 0;(atts[i] != NULL);i+=2) {
//                if( stricmp((const char*)atts[i], "file") == 0 ) {
//                    // open from file
//                    size_t len = mbstowcs(NULL, (const char*)atts[i+1], 0);
//                    wchar_t* pwname = new wchar_t[len+1];
//                    mbstowcs(pwname, (const char*)atts[i+1], len+1);
//
//                    strcpy((char*)atts[i], ""); // delete file attr
//                    _pchain->Init(pwname, (const char**)atts);
//                    break;
//                }
//                else if( stricmp((const char*)atts[i], "name") == 0 ) {
//                    _pchain->SetName((const char*)atts[i+1]);
//                }
//            }
//        }
    }
    else if( stricmp((const char*)name, "body") == 0 ) {
        LinkXMLReader* plinkreader = new LinkXMLReader(NULL, _pchain, atts);
        plinkreader->SetMassType(_masstype, _fMassValue, _vMassExtents);
        plinkreader->pKinBodyReader = this;
        _pcurreader.reset(plinkreader);
    }
    else if( stricmp((const char*)name, "joint") == 0 ) {
        _pchain->_vecJointIndices.push_back((int)_pchain->_vecJointWeights.size());
        JointXMLReader* pjointreader = new JointXMLReader(_pchain, atts);
        pjointreader->pKinBodyReader = this;
        _pcurreader.reset(pjointreader);
    }
    else if( stricmp((const char*)name, "translation") == 0 || stricmp((const char*)name, "rotationmat") == 0 ||
             stricmp((const char*)name, "rotationaxis") == 0 || stricmp((const char*)name, "quat") == 0 ||
             stricmp((const char*)name, "jointvalues") == 0 ) {
        _pcurparser = &g_realparser;
    }
    else if( stricmp((const char*)name, "mass") == 0 ) {
        // find the type of mass and create
        _masstype = LinkXMLReader::MT_Sphere;
        if( atts != NULL ) {
            for (int i = 0;(atts[i] != NULL);i++) {
                if( stricmp((const char*)atts[i], "type") == 0 ) {
                    if( stricmp((const char*)atts[i+1], "mimicgeom") == 0 ) {
                        _masstype = LinkXMLReader::MT_MimicGeom;
                    }
                    else if( stricmp((const char*)atts[i+1], "box") == 0 ) {
                        _masstype = LinkXMLReader::MT_Box;
                    }
                    else if( stricmp((const char*)atts[i+1], "sphere") == 0 ) {
                        _masstype = LinkXMLReader::MT_Sphere;
                    }

                    break;
                }
            }
        }
        _bProcessingMass = true;
    }
    else if( stricmp((const char*)name, "adjacent") == 0 ) {
        // takes two link names as arguments
        _pcurparser = &g_wstringparser;
    }
    else if( stricmp((const char*)name, "modelsdir") == 0 ) {
        // check attributes for format (default is vrml)
        _pcurparser = &g_stringparser;
    }
    else if( stricmp((const char*)name, "diffuseColor") == 0 ) {
        _pcurparser = &g_realparser;
    }
    else if( stricmp((const char*)name, "transparency") == 0 ) {
        _pcurparser = &g_floatparser;
    }
    else if( stricmp((const char*)name, "ambientColor") == 0 ) {
        _pcurparser = &g_realparser;
    }
    else {
        _pcurreader.reset(new DummyXMLReader(name, "KinBody"));
    }
}

bool KinBodyXMLReader::endElement(void *ctx ATTRIBUTE_UNUSED, const char *name)
{
    dReal* pf = _pcurparser != NULL ? (dReal*)_pcurparser->GetData() : NULL;

    if( !!_pcurreader ) {
        if( _pcurreader->endElement(ctx, name) ) {
            if( stricmp((const char*)name, "body") == 0 ) {
                KinBody::Link* plink = (KinBody::Link*)_pcurreader->Release();
                if( plink->index < 0 ) {
                    // not in array yet
                    plink->index = (int)_pchain->_veclinks.size();
                    _pchain->_veclinks.push_back(plink);
                    _vTransforms.push_back(Transform());
                }

                // do this later, or else offsetfrom will be messed up!
                _vTransforms[plink->GetIndex()] = dynamic_cast<LinkXMLReader*>(_pcurreader.get())->GetOrigTransform();
            }
            else if( stricmp((const char*)name, "joint") == 0 ) {
                KinBody::Joint* pnewjoint = (KinBody::Joint*)_pcurreader->Release();
                pnewjoint->dofindex = (int)_pchain->_vecJointIndices.back();
                
                assert( pnewjoint->dofindex < _pchain->GetDOF());

                if( ((JointXMLReader*)_pcurreader.get())->IsMimic() )
                    listMimicJoints.push_back(pair<KinBody::Joint*,string>(pnewjoint,((JointXMLReader*)_pcurreader.get())->GetMimicJoint()));

                if( ((JointXMLReader*)_pcurreader.get())->IsDisabled() ) {
                    for(int i = 0; i < pnewjoint->GetDOF(); ++i)
                        _pchain->_vecJointWeights.pop_back();
                    _pchain->_vecJointIndices.pop_back();
                    pnewjoint->jointindex = (int)_pchain->_vecPassiveJoints.size();
                    pnewjoint->dofindex = -1;
                    _pchain->_vecPassiveJoints.push_back(pnewjoint);
                }
                else {
                    pnewjoint->jointindex = (int)_pchain->_vecjoints.size();
                    _pchain->_vecjoints.push_back(pnewjoint);
                }
            }
            else if( stricmp((const char*)name, "kinbody") == 0 ) {
                _pcurreader->Release();

                // most likely new transforms were added, so update
                _pchain->GetBodyTransformations(_vTransforms);
            }
            else
                RAVELOG_INFOA("releasing unknown tag %s\n",name);

            _pcurreader.reset();
        }
    }
    else if( _bProcessingMass ) {
        if( stricmp((const char*)name, "density") == 0 ) {
            if( _masstype == LinkXMLReader::MT_BoxMass ) _masstype = LinkXMLReader::MT_Box;
            else if( _masstype == LinkXMLReader::MT_SphereMass) _masstype = LinkXMLReader::MT_Sphere;
            _fMassValue = pf[0];
        }
        else if( stricmp((const char*)name, "total") == 0 ) {
            if( _masstype == LinkXMLReader::MT_Box ) _masstype = LinkXMLReader::MT_BoxMass;
            else if( _masstype == LinkXMLReader::MT_Sphere) _masstype = LinkXMLReader::MT_SphereMass;
            _fMassValue = pf[0];
        }
        else if( stricmp((const char*)name, "radius") == 0 ) {
            _vMassExtents.x = pf[0];
        }
        else if( _masstype == LinkXMLReader::MT_Box && stricmp((const char*)name, "extents") == 0 ) {
            assert( _pcurparser->GetCount() == 3 );
            _vMassExtents = Vector(pf[0], pf[1], pf[2]);
        }
        else if( stricmp((const char*)name, "mass") == 0 ) {
            assert( _bProcessingMass );
            _bProcessingMass = false;
        }
    }
    else if( InterfaceXMLReader::endElement(ctx,name) ) {
        // go through all mimic joints and assign the correct indices
        FOREACH(itmimic, listMimicJoints) {
            itmimic->first->nMimicJointIndex = _pchain->GetJointIndex(_ravembstowcs(itmimic->second.c_str()).c_str());
            if( itmimic->first->nMimicJointIndex < 0 ) {
                RAVEPRINT(L"Failed to find mimic joint: %s", itmimic->second.c_str());
                g_XMLErrorCount++;
            }
        }
                
        if( _bodyname.size() > 0 )
            _pchain->SetName(_bodyname.c_str());

        // add prefix
        if( prefix.size() > 0 ) {
            vector<KinBody::Link*>::iterator itlink = _pchain->_veclinks.begin()+rootoffset;
            while(itlink != _pchain->_veclinks.end()) {
                (*itlink)->name = wprefix + (*itlink)->name;
                ++itlink;
            }
            vector<KinBody::Joint*>::iterator itjoint = _pchain->_vecjoints.begin()+rootjoffset;
            while(itjoint != _pchain->_vecjoints.end()) {
                (*itjoint)->name = wprefix +(*itjoint)->name;
                ++itjoint;
            }
        }

        if( _bOverwriteDiffuse ) {
            // overwrite the color
            FOREACH(itlink, _pchain->_veclinks) {
                FOREACH(itprop, (*itlink)->_listGeomProperties)
                    itprop->diffuseColor = _diffusecol;
            }
        }
        if( _bOverwriteAmbient ) {
            // overwrite the color
            FOREACH(itlink, _pchain->_veclinks) {
                FOREACH(itprop, (*itlink)->_listGeomProperties)
                    itprop->ambientColor = _ambientcol;
            }
        }
        if( _bOverwriteTransparency ) {
            // overwrite the color
            FOREACH(itlink, _pchain->_veclinks) {
                FOREACH(itprop, (*itlink)->_listGeomProperties)
                    itprop->ftransparency = _transparency;
            }
        }

        // transform all the bodies with trans
        Transform cur;

        //RAVEPRINT(L"kinbody: %d %"PRIdS"\n", rootoffset, _pchain->_veclinks.size());

        vector<KinBody::Link*>::iterator itlink = _pchain->_veclinks.begin()+rootoffset;
        while(itlink != _pchain->_veclinks.end()) {
            (*itlink)->SetTransform(_trans * (*itlink)->GetTransform());
            ++itlink;
        }

        //_pchain->RestoreJoints();

        if( _vjointvalues.size() > 0 ) {
            if( _vjointvalues.size() == _pchain->GetJoints().size() )
                _pchain->SetJointValues(NULL, NULL, &_vjointvalues[0]);
            else
                RAVELOG_WARNA("jointvalues for body %S wrong number (%"PRIdS"!=%"PRIdS")\n", _pchain->GetName(), _vjointvalues.size(), _pchain->GetJoints().size());
        }
        
        Vector com = _pchain->GetCenterOfMass();
        RAVELOGA("%S: COM = (%f,%f,%f)\n", _pchain->GetName(), com.x, com.y, com.z);

        // output AABB per link
        //        RAVEPRINT(L"processing\n");
        //        wstringstream ss; ss << "Links: " << endl;
        //        int linkindex = 0;
        //        FORIT(itlink, _pchain->_veclinks) {
        //            Transform t = (*itlink)->GetTransform();
        //            AABB ab = (*itlink)->ComputeAABB();
        //            ab.pos = t.inverse()*ab.pos;
        //            ss << "      <Geom name = \"" << (*itlink)->GetName() << "\" type=\"box\">" << endl
        //                << "        <extents>" << ab.extents.x << " " << ab.extents.y << " " << ab.extents.z << "</extents>" << endl
        //                << "        <translation>" << ab.pos.x << " " << ab.pos.y << " " << ab.pos.z << "</translation>" << endl
        //                << "      </Geom>" << endl;
        //        }
        //        RAVEPRINT(ss.str().c_str());
        return true;
    }
    else if( stricmp((const char*)name, "translation") == 0 ) {
        assert( _pcurparser->GetCount() == 3 );
        _trans.trans = Vector(pf[0], pf[1], pf[2]);
    }
    else if( stricmp((const char*)name, "rotationaxis") == 0 ) {
        // check if quaternion or rotation matrix
        assert( _pcurparser->GetCount() == 4 );
        
        Transform tnew;
        normalize3(pf, pf);
        tnew.rotfromaxisangle(Vector(pf[0], pf[1], pf[2]), pf[3] * PI / 180.0f);
        _trans.rot = (tnew*_trans).rot;
    }
    else if( stricmp((const char*)name, "quat") == 0 ) {
        // check if quaternion or rotation matrix
        assert( _pcurparser->GetCount() == 4 );
        
        Transform tnew;
        normalize4(tnew.rot, pf);
        _trans.rot = (tnew*_trans).rot;
    }
    else if( stricmp((const char*)name, "rotationmat") == 0 ) {
        // check if quaternion or rotation matrix
        assert( _pcurparser->GetCount() == 9 );

        TransformMatrix tnew;
        tnew.m[0] = pf[0]; tnew.m[1] = pf[1]; tnew.m[2] = pf[2]; tnew.m[3] = 0;
        tnew.m[4] = pf[3]; tnew.m[5] = pf[4]; tnew.m[6] = pf[5]; tnew.m[7] = 0;
        tnew.m[8] = pf[6]; tnew.m[9] = pf[7]; tnew.m[10] = pf[8]; tnew.m[11] = 0;
        _trans.rot = (Transform(tnew)*_trans).rot;
    }
    else if( stricmp((const char*)name, "adjacent") == 0 ) {
        if( _pcurparser->GetData() != NULL ) {
            wchar_t* ptr=NULL;
            wchar_t* p = WCSTOK((wchar_t*)_pcurparser->GetData(), L" ,\t", &ptr);
            pair<wstring, wstring> entry;

            if(p != NULL) {
                entry.first = p;
                p = WCSTOK(NULL, L" ,\t", &ptr);
                if( p != NULL ) {
                    entry.second = p;
                    _pchain->_vForcedAdjacentLinks.push_back(entry);
                }
            }
        }
    }
    else if( stricmp((const char*)name, "modelsdir") == 0 ) {
        _strModelsDir = (const char*)_pcurparser->GetData();
        _strModelsDir += "/";
    }
    else if( stricmp((const char*)name, "diffuseColor") == 0 ) {
        // check attributes for format (default is vrml)
        _diffusecol = Vector(pf[0], pf[1], pf[2]);
        _bOverwriteDiffuse = true;
    }
    else if( stricmp((const char*)name, "ambientColor") == 0 ) {
        // check attributes for format (default is vrml)
        _ambientcol = Vector(pf[0], pf[1], pf[2]);
        _bOverwriteAmbient = true;
    }
    else if( stricmp((const char*)name, "transparency") == 0 ) {
        _transparency = *(float*)_pcurparser->GetData();
        _bOverwriteTransparency = true;
    }
    else if( stricmp((const char*)name, "jointvalues") == 0 ) {
        _vjointvalues.resize(_pcurparser->GetCount());
        for(int i = 0; i < _pcurparser->GetCount(); ++i)
            _vjointvalues[i] = pf[i];
    }

    _pcurparser = NULL;
    return false;
}

void KinBodyXMLReader::characters(void *ctx ATTRIBUTE_UNUSED, const char *ch, int len)
{
    if( !!_pcurreader )
        _pcurreader->characters(ctx, ch, len);
    else if( _pcurparser != NULL )
        _pcurparser->Format((const char*)ch, len);
    else
        InterfaceXMLReader::characters(ctx,ch,len);
}

//// Manipulator Reader ////
ManipulatorXMLReader::ManipulatorXMLReader(RobotBase::Manipulator* pmanip, RobotBase* probot, const char **atts)
{
    assert( probot != NULL );
    _pmanip = pmanip;

    if( _pmanip == NULL )
        _pmanip = new RobotBase::Manipulator(probot);

    if( atts != NULL ) {
        for (int i = 0;(atts[i] != NULL);i+=2) {
            if( stricmp((const char*)atts[i], "name") == 0 ) {
                _pmanip->_name = (const char*)atts[i+1];
            }
        }
    }

    _probot = probot;
    _pcurparser = NULL;
}

void ManipulatorXMLReader::startElement(void *ctx ATTRIBUTE_UNUSED, const char *name, const char **atts)
{
    if( !!_pcurreader ) {
        _pcurreader->startElement(ctx, name, atts);
    }
    else if( stricmp((const char*)name, "effector") == 0 ||
        stricmp((const char*)name, "joints") == 0 ||
        stricmp((const char*)name, "armjoints") == 0 ||
        stricmp((const char*)name, "base") == 0) {
        _pcurparser = &g_wstringparser;
    }
    else if( stricmp((const char*)name, "iksolver") == 0 ) {
        _pcurparser = &g_stringparser;
    }
    else if( stricmp((const char*)name, "opened") == 0 ||
             stricmp((const char*)name, "closed") == 0 ||
             stricmp((const char*)name, "translation") == 0 ||
             stricmp((const char*)name, "quat") == 0 ||
             stricmp((const char*)name, "rotationaxis") == 0 ||
             stricmp((const char*)name, "rotationmat") == 0) {
        _pcurparser = &g_realparser;
    }
    else {
        _pcurreader.reset(new DummyXMLReader(name, "Manipulator"));
    }
}

/// if returns true, XMLReader has finished parsing
bool ManipulatorXMLReader::endElement(void *ctx ATTRIBUTE_UNUSED, const char *name)
{
    if( !!_pcurreader ) {
        if( _pcurreader->endElement(ctx, name) ) {
            _pcurreader.reset();
        }
    }
    else if( stricmp((const char*)name, "manipulator") == 0 ) {

        if( _pmanip->_pIkSolver != NULL )
            _pmanip->_pIkSolver->Init(_probot, _pmanip, _pmanip->_ikoptions);
        if( _pmanip->_vecjoints.size() != _pmanip->_vClosedGrasp.size() || _pmanip->_vecjoints.size() != _pmanip->_vOpenGrasp.size() ) {
            RAVELOG(L"Manipulator has open/closed grasps wrong\n");
            _pmanip->_vClosedGrasp.clear();
            _pmanip->_vOpenGrasp.clear();
        }
        return true;
    }
    else if( stricmp((const char*)name, "effector") == 0 ) {
        // look up the correct link
        // figure out which body
        _pmanip->pEndEffector = _probot->GetLink((const wchar_t*)_pcurparser->GetData());
        
        if( _pmanip->pEndEffector == NULL ) {
            RAVEPRINT(L"Failed to find manipulator end effector %ls\n", _pcurparser->GetData());
            g_XMLErrorCount++;
        }
    }
    else if( stricmp((const char*)name, "base") == 0 ) {
        // look up the correct link
        // figure out which body
        _pmanip->pBase = _probot->GetLink((const wchar_t*)_pcurparser->GetData());
        
        if( _pmanip->pBase == NULL ) {
            RAVEPRINT(L"Failed to find manipulator base %ls\n", _pcurparser->GetData());
            g_XMLErrorCount++;
        }
    }
    else if( stricmp((const char*)name, "joints") == 0 ) {
        
        if( _pcurparser->GetData() != NULL ) {
            wchar_t* ptr=NULL;
            wchar_t* p = WCSTOK((wchar_t*)_pcurparser->GetData(), L" ,\t", &ptr);
            
            while(p != NULL) {
                int index = _probot->GetJointIndex(p);
                if( index < 0 ) {
                    RAVEPRINT(L"Failed to find joint %ls for manipulator\n", p);
                    g_XMLErrorCount++;
                }
                else
                    _pmanip->_vecjoints.push_back(_probot->GetJoint(index)->GetDOFIndex());
                p = WCSTOK(NULL, L" ,\t", &ptr);
            }
        }
    }
    else if( stricmp((const char*)name, "armjoints") == 0 ) {
        
        if( _pcurparser->GetData() != NULL ) {
            wchar_t* ptr=NULL;
            wchar_t* p = WCSTOK((wchar_t*)_pcurparser->GetData(), L" ,\t", &ptr);
            
            while(p != NULL) {
                int index = _probot->GetJointIndex(p);
                if( index < 0 ) {
                    RAVEPRINT(L"Failed to find joint %ls for manipulator\n", p);
                    g_XMLErrorCount++;
                }
                else
                    _pmanip->_vecarmjoints.push_back(_probot->GetJoint(index)->GetDOFIndex());
                p = WCSTOK(NULL, L" ,\t", &ptr);
            }
        }
    }
    else if( stricmp((const char*)name, "iksolver") == 0 ) {
        if( _pcurparser->GetData() != NULL ) {
            stringstream ss((char*)_pcurparser->GetData());
            string ikname;
            ss >> ikname >> _pmanip->_ikoptions;
            
            IkSolverBase* piksolver = _probot->GetEnv()->CreateIkSolver(ikname.c_str());
            if( piksolver == NULL ) {
                RAVELOG(L"failed to create iksolver %s\n", ikname.c_str());
                _pmanip->_strIkSolver = ikname;
            }
            else {
                _pmanip->SetIKSolver(piksolver);
                // don't init here
            }
        }
    }
    else if( stricmp((const char*)name, "opened") == 0 ) {
        _pmanip->_vOpenGrasp.resize(_pcurparser->GetCount());
        dReal* pf = (dReal*)_pcurparser->GetData();
        for(size_t i = 0; i < _pmanip->_vOpenGrasp.size(); ++i)
            _pmanip->_vOpenGrasp[i] = pf[i] * PI/180;
    }
    else if( stricmp((const char*)name, "closed") == 0 ) {
        _pmanip->_vClosedGrasp.resize(_pcurparser->GetCount());
        dReal* pf = (dReal*)_pcurparser->GetData();
        for(size_t i = 0; i < _pmanip->_vClosedGrasp.size(); ++i)
            _pmanip->_vClosedGrasp[i] = pf[i] * PI/180;
    }
    else if( stricmp((const char*)name, "translation") == 0 ) {
        dReal* pf = _pcurparser != NULL ? (dReal*)_pcurparser->GetData() : NULL;
        _pmanip->tGrasp.trans = Vector(pf[0], pf[1], pf[2]);
    }
    else if( stricmp((const char*)name, "quat") == 0 ) {
        Transform tnew;
        normalize4(tnew.rot, (dReal*)_pcurparser->GetData());
        _pmanip->tGrasp.rot = (tnew*_pmanip->tGrasp).rot;
    }
    else if( stricmp((const char*)name, "rotationaxis") == 0 ) {
        dReal* pf = (dReal*)_pcurparser->GetData();
        normalize3(pf, pf);
        Transform tnew;
        tnew.rotfromaxisangle(Vector(pf[0], pf[1], pf[2]), pf[3] * PI / 180.0f);
        _pmanip->tGrasp.rot = (tnew*_pmanip->tGrasp).rot;
    }
    else if( stricmp((const char*)name, "rotationmat") == 0 ) {
        dReal* pf = (dReal*)_pcurparser->GetData();

        TransformMatrix tnew;
        tnew.m[0] = pf[0]; tnew.m[1] = pf[1]; tnew.m[2] = pf[2]; tnew.m[3] = 0;
        tnew.m[4] = pf[3]; tnew.m[5] = pf[4]; tnew.m[6] = pf[5]; tnew.m[7] = 0;
        tnew.m[8] = pf[6]; tnew.m[9] = pf[7]; tnew.m[10] = pf[8]; tnew.m[11] = 0;
        _pmanip->tGrasp.rot = (Transform(tnew)*_pmanip->tGrasp).rot;
    }
    
    _pcurparser = NULL;
    return false;
}

void ManipulatorXMLReader::characters(void *ctx ATTRIBUTE_UNUSED, const char *ch, int len)
{
    if( !!_pcurreader )
        _pcurreader->characters(ctx, ch, len);
    else if( _pcurparser != NULL )
        _pcurparser->Format((const char*)ch, len);
}

//// Manipulator Reader ////
AttachedSensorXMLReader::AttachedSensorXMLReader(RobotBase::AttachedSensor* psensor, RobotBase* probot, const char **atts)
{
    assert( probot != NULL );
    _psensor = psensor;

    if( _psensor == NULL )
        _psensor = new RobotBase::AttachedSensor(probot);

    if( atts != NULL ) {
        for (int i = 0;(atts[i] != NULL);i+=2) {
            if( stricmp((const char*)atts[i], "name") == 0 ) {
                _psensor->_name = (const char*)atts[i+1];
            }
        }
    }

    _probot = probot;
    _pcurparser = NULL;
}

void AttachedSensorXMLReader::startElement(void *ctx ATTRIBUTE_UNUSED, const char *name, const char **atts)
{
    if( !!_pcurreader ) {
        _pcurreader->startElement(ctx, name, atts);
    }
    else if( stricmp((const char*)name, "link") == 0 ) {
        _pcurparser = &g_wstringparser;
    }
    else if( stricmp((const char*)name, "translation") == 0 ||
             stricmp((const char*)name, "quat") == 0 ||
             stricmp((const char*)name, "rotationaxis") == 0 ||
             stricmp((const char*)name, "rotationmat") == 0 ) {
        _pcurparser = &g_realparser;
    }
    else if( stricmp((const char*)name, "sensor") == 0 ) {
        // create the sensor
        delete _psensor->psensor; _psensor->psensor = NULL;

        const char* ptype = NULL;
        const char* pname = NULL;

        if( atts != NULL ) {
            for (int i = 0;(atts[i] != NULL);i+=2) {
                if( stricmp((const char*)atts[i], "type") == 0 ) {
                    ptype = (const char*)atts[i+1];
                }
                else if( stricmp((const char*)atts[i], "args") == 0 )
                    args = (const char*)atts[i+1];
                else if( stricmp((const char*)atts[i], "name") == 0 )
                    pname = (const char*)atts[i+1];
            }
        }

        if( ptype != NULL ) {
            _psensor->psensor = _probot->GetEnv()->CreateSensor(_ravembstowcs(ptype).c_str());

            if( _psensor->psensor == NULL ) {
                RAVEPRINT(L"failed to create sensor %s\n", ptype);
            }
            else {
                _pcurreader.reset(_psensor->psensor->CreateXMLReader());
            }
        }

        if( _psensor->psensor != NULL )
            _psensor->psensor->SetName(pname);

        if( !_pcurreader ) {
            // create a dummy
            _pcurreader.reset(new DummyXMLReader(name, "AttachedSensor"));
        }
    }
    else {
        _pcurreader.reset(new DummyXMLReader(name, "AttachedSensor"));
    }
}

/// if returns true, XMLReader has finished parsing
bool AttachedSensorXMLReader::endElement(void *ctx ATTRIBUTE_UNUSED, const char *name)
{
    if( !!_pcurreader ) {
        if( _pcurreader->endElement(ctx, name) ) {
            _pcurreader->Release();
            _pcurreader.reset();
        }
    }
    else if( stricmp((const char*)name, "attachedsensor") == 0 ) {
        if( _psensor->psensor == NULL ) {
            RAVEPRINT(L"Attached robot sensor points to no real sensor!\n");
        }
        else {
            if( !_psensor->psensor->Init(args.c_str()) ) {
                RAVEPRINT(L"failed to initialize sensor %s\n", _psensor->GetName());
                delete _psensor->psensor; _psensor->psensor = NULL;
            }
            else {
                _psensor->pdata = _psensor->psensor->CreateSensorData();
                
                if( _psensor->pattachedlink == NULL ) {
                    RAVELOG(L"attached link is NULL, setting to base of robot\n");
                    if( _probot->GetLinks().size() == 0 ) {
                        RAVEPRINT(L"robot has no links!\n");
                        _psensor->pattachedlink = NULL;
                    }
                    else
                        _psensor->pattachedlink = _probot->GetLinks().front();
                }
            }
        }

        return true;
    }
    else if( stricmp((const char*)name, "link") == 0 ) {
        // look up the correct link
        // figure out which body
        _psensor->pattachedlink = _probot->GetLink((const wchar_t*)_pcurparser->GetData());
        
        if( _psensor->pattachedlink == NULL ) {
            RAVEPRINT(L"Failed to find attached sensor link %S\n", _pcurparser->GetData());
            g_XMLErrorCount++;
        }
    }
    else if( stricmp((const char*)name, "translation") == 0 ) {
        dReal* pf = _pcurparser != NULL ? (dReal*)_pcurparser->GetData() : NULL;
        _psensor->trelative.trans = Vector(pf[0], pf[1], pf[2]);
    }
    else if( stricmp((const char*)name, "quat") == 0 ) {
        Transform tnew;
        normalize4(tnew.rot, (dReal*)_pcurparser->GetData());
        _psensor->trelative.rot = (tnew*_psensor->trelative).rot;
    }
    else if( stricmp((const char*)name, "rotationaxis") == 0 ) {
        dReal* pf = (dReal*)_pcurparser->GetData();
        normalize3(pf, pf);
        Transform tnew;
        tnew.rotfromaxisangle(Vector(pf[0], pf[1], pf[2]), pf[3]*PI/180.0f);
        _psensor->trelative.rot = (tnew*_psensor->trelative).rot;
    }
    else if( stricmp((const char*)name, "rotationmat") == 0 ) {
        dReal* pf = (dReal*)_pcurparser->GetData();
        TransformMatrix tnew;
        tnew.m[0] = pf[0]; tnew.m[1] = pf[1]; tnew.m[2] = pf[2]; tnew.m[3] = 0;
        tnew.m[4] = pf[3]; tnew.m[5] = pf[4]; tnew.m[6] = pf[5]; tnew.m[7] = 0;
        tnew.m[8] = pf[6]; tnew.m[9] = pf[7]; tnew.m[10] = pf[8]; tnew.m[11] = 0;
        _psensor->trelative.rot = (Transform(tnew)*_psensor->trelative).rot;
    }
    
    _pcurparser = NULL;
    return false;
}

void AttachedSensorXMLReader::characters(void *ctx ATTRIBUTE_UNUSED, const char *ch, int len)
{
    if( !!_pcurreader )
        _pcurreader->characters(ctx, ch, len);
    else if( _pcurparser != NULL )
        _pcurparser->Format((const char*)ch, len);
}


//// Robot Reader ////
RobotXMLReader::RobotXMLReader(Environment* penv, RobotBase* probot, const char **atts) : InterfaceXMLReader(penv,PT_Robot,probot,"robot",atts)
{
    _probot = (RobotBase*)_pinterface;
    _pcurparser = NULL;
    bRobotInit = false;
    pNewController = NULL;
    assert( _probot != NULL );
    rootoffset = rootjoffset = rootsoffset = rootmoffset = 0;
    if( probot != NULL ) {
        rootoffset = (int)probot->GetLinks().size();
        rootjoffset = (int)probot->GetJoints().size();
        rootsoffset = (int)probot->GetSensors().size();
        rootmoffset = (int)probot->GetManipulators().size();
    }

    if( atts != NULL ) {
        for (int i = 0;(atts[i] != NULL);i+=2) {
            if( stricmp((const char*)atts[i], "name") == 0 ) {
                _robotname = (const char*)atts[i+1];
            }
            else if( stricmp((const char*)atts[i], "prefix") == 0 ) {
                prefix = (const char*)atts[i+1];
                wprefix = _ravembstowcs((const char*)atts[i+1]);
            }
        }
    }
}

void RobotXMLReader::startElement(void *ctx ATTRIBUTE_UNUSED, const char *name, const char **atts)
{
    assert( _probot != NULL );

    if( !!_pcurreader ) {
        _pcurreader->startElement(ctx, name, atts);
        return;
    }

    InterfaceXMLReader::startElement(ctx,name,atts);
    if( !!_pcustomreader )
        return;

    if( stricmp((const char*)name, "robot") == 0 ) {
        //const char* ptype = NULL;
        _pcurreader.reset(new RobotXMLReader(_penv, _probot, atts));
//        if( atts != NULL ) {
//            for (int i = 0;(atts[i] != NULL);i+=2) {
//                if( stricmp((const char*)atts[i], "file") == 0 ) {
//                    // open from file
//                    strcpy((char*)atts[i], ""); // delete file attr
//                    if( (_probot = g_Environ.ReadRobotXML(_probot, (const char*)atts[i+1], (const char**)atts)) == NULL ) {
//                        break;
//                        RAVELOGA("Failed to load robot filename %s\n", atts[i+1]);
//                    }
//                    else
//                        bRobotInit = true;
//                }
//                else if( stricmp((const char*)atts[i], "name") == 0 ) {
//                    _probot->SetName((const char*)atts[i+1]);
//                }
//                else if( stricmp((const char*)atts[i], "type") == 0 ) {
//                    ptype = (const char*)atts[i+1];
//                }
//            }
//        }
//
//        if( _probot == NULL ) {
//            // create the robot
//            if( ptype != NULL ) {
//                _probot = g_Environ.CreateRobot(_ravembstowcs(ptype).c_str());
//                if( _probot == NULL )
//                    RAVELOGA("Failed to find robot %s\n", ptype);
//            }
//            else _probot = g_Environ.CreateRobot(L"GenericRobot");
//        }
//        else {
//            // if ptype != NULL and gives a valid robot, use it instead
//            if( ptype != NULL ) {
//                RobotBase* ptemp = g_Environ.CreateRobot(_ravembstowcs(ptype).c_str());
//                if( ptemp != NULL ) {
//                    delete _probot;
//                    _probot = ptemp;
//                }
//            }
//        }
//
//        if( _probot == NULL ) {
//            RAVELOGA("Failed to find GenericRobot\n");
//            _probot = (RobotBase*)_penv->CreateInterface(PT_Robot, "");
//        }
    }
    else if( stricmp((const char*)name, "kinbody") == 0 ) {
        _pcurreader.reset(new KinBodyXMLReader(_penv,_probot->GetInterfaceType(),_probot, atts));
    }
    else if( stricmp((const char*)name, "manipulator") == 0 ) {
        _probot->_vecManipulators.push_back(RobotBase::Manipulator(_probot));
        _pcurreader.reset(new ManipulatorXMLReader(&_probot->_vecManipulators.back(), _probot, atts));
    }
    else if( stricmp((const char*)name, "attachedsensor") == 0 ) {
        _probot->_vecSensors.push_back(RobotBase::AttachedSensor(_probot));
        _pcurreader.reset(new AttachedSensorXMLReader(&_probot->_vecSensors.back(), _probot, atts));
    }
    else if( stricmp((const char*)name, "controller") == 0 ) {
        _pcurparser = &g_stringparser;
    }
    else if( stricmp((const char*)name, "translation") == 0 || stricmp((const char*)name, "rotationmat") == 0 ||
        stricmp((const char*)name, "rotationaxis") == 0 || stricmp((const char*)name, "quat") == 0 ||
             stricmp((const char*)name, "jointvalues") == 0) {
        _pcurparser = &g_realparser;
    }
    else {
        _pcurreader.reset(new DummyXMLReader(name, "Robot"));
    }
}

/// if returns true, XMLReader has finished parsing
bool RobotXMLReader::endElement(void *ctx ATTRIBUTE_UNUSED, const char *name)
{
    dReal* pf = _pcurparser != NULL ? (dReal*)_pcurparser->GetData() : NULL;

    if( !!_pcurreader ) {
        if( _pcurreader->endElement(ctx, name) ) {
            _pcurreader->Release();
            _pcurreader.reset();
        }
    }
    else if( InterfaceXMLReader::endElement(ctx,name) ) {
        if( _robotname.size() > 0 )
            _probot->SetName(_robotname.c_str());

        // add prefix
        if( prefix.size() > 0 ) {
            vector<KinBody::Link*>::iterator itlink = _probot->_veclinks.begin()+rootoffset;
            while(itlink != _probot->_veclinks.end()) {
                (*itlink)->name = wprefix + (*itlink)->name;
                ++itlink;
            }
            vector<KinBody::Joint*>::iterator itjoint = _probot->_vecjoints.begin()+rootjoffset;
            while(itjoint != _probot->_vecjoints.end()) {
                (*itjoint)->name = wprefix +(*itjoint)->name;
                ++itjoint;
            }
            vector<RobotBase::AttachedSensor>::iterator itsensor = _probot->GetSensors().begin()+rootsoffset;
            while(itsensor != _probot->GetSensors().end()) {
                itsensor->_name = prefix + itsensor->_name;
                ++itsensor;
            }
            vector<RobotBase::Manipulator>::iterator itmanip = _probot->GetManipulators().begin()+rootmoffset;
            while(itmanip != _probot->GetManipulators().end()) {
                itmanip->_name = prefix + itmanip->_name;
                ++itmanip;
            }
        }
        
        // transform all "new" bodies with trans
        vector<KinBody::Link*>::iterator itlink = _probot->_veclinks.begin()+rootoffset;
        while(itlink != _probot->_veclinks.end()) {
            (*itlink)->SetTransform(_trans * (*itlink)->GetTransform());
            ++itlink;
        }
        
        if( _vjointvalues.size() > 0 ) {
            if( _vjointvalues.size() == _probot->GetJoints().size() ) {
                _probot->SetJointValues(NULL, NULL, &_vjointvalues[0]);
            }
            else
                RAVELOG_WARNA("jointvalues for body %S wrong number (%"PRIdS"!=%"PRIdS")\n", _probot->GetName(), _vjointvalues.size(), _probot->GetJoints().size());
        }

        if( pNewController != NULL ) {
            RAVELOG(L"setting controller %s, args=%s\n", pNewController->GetXMLId(), strControllerArgs.c_str());
            _probot->SetController(pNewController, strControllerArgs.c_str());
        }
        else if( _probot->GetController() == NULL ) {
            // create a default controller
            ControllerBase* pcontrol = _probot->GetEnv()->CreateController(L"IdealController");
            if( pcontrol == NULL )
                pcontrol = _probot->GetEnv()->CreateController(L"ControllerPD");
            _probot->SetController(pcontrol);
        }
        else {
            if( !bRobotInit && _probot->GetController() != NULL && _probot->GetDOF() > 0 ) {
                _probot->GetController()->Init(_probot);
                //vector<dReal> vals;
                //_probot->GetJointValues(vals);
                //_probot->GetController()->SetDesired(&vals[0]);
            }
        }

        // forces robot to reupdate its internal objects
        _probot->SetTransform(_probot->GetTransform());
        
        if( !_probot->GetEnv()->GetPhysicsEngine()->InitKinBody(_probot) )
            RAVELOG(L"physics engine failed to init robot %S\n", _probot->GetName());
        return true;
    }
    else if( stricmp((const char*)name, "translation") == 0 ) {
        assert( _pcurparser->GetCount() == 3 );
        _trans.trans = Vector(pf[0], pf[1], pf[2]);
    }
    else if( stricmp((const char*)name, "rotationaxis") == 0 ) {
        // check if quaternion or rotation matrix
        assert( _pcurparser->GetCount() == 4 );
        
        Transform tnew;
        normalize3(pf, pf);
        tnew.rotfromaxisangle(Vector(pf[0], pf[1], pf[2]), pf[3] * PI / 180.0f);
        _trans.rot = (tnew*_trans).rot;
    }
    else if( stricmp((const char*)name, "quat") == 0 ) {
        // check if quaternion or rotation matrix
        assert( _pcurparser->GetCount() == 4 );
        
        Transform tnew;
        normalize4(tnew.rot, pf);
        _trans.rot = (tnew*_trans).rot;
    }
    else if( stricmp((const char*)name, "rotationmat") == 0 ) {
        // check if quaternion or rotation matrix
        assert( _pcurparser->GetCount() == 9 );
        
        TransformMatrix tnew;
        tnew.m[0] = pf[0]; tnew.m[1] = pf[1]; tnew.m[2] = pf[2]; tnew.m[3] = 0;
        tnew.m[4] = pf[3]; tnew.m[5] = pf[4]; tnew.m[6] = pf[5]; tnew.m[7] = 0;
        tnew.m[8] = pf[6]; tnew.m[9] = pf[7]; tnew.m[10] = pf[8]; tnew.m[11] = 0;
        _trans.rot = (Transform(tnew)*_trans).rot;
    }
    else if( stricmp((const char*)name, "controller") == 0 ) {

        char* pstr = (char*)_pcurparser->GetData();
        // format: controllername args
        char* args = strchr(pstr, ' ');
        if( args != NULL ) {
            *args++ = 0;
            strControllerArgs = args;
        }
        else
            strControllerArgs = "";

        pNewController = _probot->GetEnv()->CreateController(_ravembstowcs(pstr).c_str());

        if( pNewController == NULL ) {
            RAVELOGA("Failed to find controller %s\n", pstr);
            g_XMLErrorCount++;
        }
    }
    else if( stricmp((const char*)name, "jointvalues") == 0 ) {
        _vjointvalues.resize(_pcurparser->GetCount());
        for(int i = 0; i < _pcurparser->GetCount(); ++i)
            _vjointvalues[i] = pf[i];
    }

    _pcurparser = NULL;
    return false;
}

void RobotXMLReader::characters(void *ctx ATTRIBUTE_UNUSED, const char *ch, int len)
{
    if( !!_pcurreader )
        _pcurreader->characters(ctx, ch, len);
    else if( _pcurparser != NULL )
        _pcurparser->Format((const char*)ch, len);
    else
        InterfaceXMLReader::characters(ctx,ch,len);
}

ControllerXMLReader::ControllerXMLReader(Environment* penv, InterfaceBase* pinterface, const char** atts) : InterfaceXMLReader(penv,PT_Controller,pinterface,g_mapInterfaceNames[PT_Controller],atts)
{
    if( atts != NULL ) {
        for (int i = 0;(atts[i] != NULL);i+=2) {
            if( stricmp((const char*)atts[i], "robot") == 0 ) {
                _robotname = _ravembstowcs((const char*)atts[i+1]);
            }
            else if( stricmp((const char*)atts[i], "args") == 0 ) {
                _args = (const char*)atts[i+1];
            }
        }
    }
}

void ControllerXMLReader::startElement(void *ctx ATTRIBUTE_UNUSED, const char *name, const char **atts)
{
    if( !!_pcurreader ) {
        _pcurreader->startElement(ctx, name, atts);
        return;
    }

    InterfaceXMLReader::startElement(ctx,name,atts);
    if( !!_pcustomreader )
        return;

    _pcurreader.reset(new DummyXMLReader(name, g_mapInterfaceNames[PT_Controller].c_str()));
}

bool ControllerXMLReader::endElement(void *ctx ATTRIBUTE_UNUSED, const char *name)
{
    if( !!_pcurreader ) {
        if( _pcurreader->endElement(ctx, name) )
            _pcurreader.reset();
    }
    else if( InterfaceXMLReader::endElement(ctx,name) ) {
        RobotBase* probot = NULL;
        if( _robotname.size() > 0 ) {
            KinBody* pbody = _penv->GetKinBody(_robotname.c_str());
            if( pbody->IsRobot() )
                probot = (RobotBase*)pbody;
        }

        if( probot )
            probot->SetController((ControllerBase*)_pinterface,_args.c_str());
        else
            RAVELOG_WARNA("controller is unused\n");
        return true;
    }
    return false;
}

void ControllerXMLReader::characters(void *ctx ATTRIBUTE_UNUSED, const char *ch, int len)
{
    if( !!_pcurreader )
        _pcurreader->characters(ctx,ch,len);
    else
        InterfaceXMLReader::characters(ctx,ch,len);
}

//// Environment Reader ////
EnvironmentXMLReader::EnvironmentXMLReader(Environment* penv, const char **atts)
{
    _penv = penv;
    if( _penv == NULL )
        _penv = new Environment;
    _pcurparser = NULL;
    _bInEnvironment = false;

    tCamera.trans = Vector(-0.5f, 1.5f, 0.8f);
    tCamera.rotfromaxisangle(Vector(1, 0, 0), (dReal)-0.5);
    vBkgndColor = Vector(1,1,1);
    bTransSpecified = false;
}

EnvironmentXMLReader::~EnvironmentXMLReader()
{
    delete _penv;
}

void EnvironmentXMLReader::startElement(void *ctx ATTRIBUTE_UNUSED, const char *name, const char **atts)
{
    if( !!_pcurreader ) {
        _pcurreader->startElement(ctx, name, atts);
        return;
    }

    // check for any plugins
    FOREACH(itname,g_mapInterfaceNames) {
        if( stricmp((const char*)name, itname->second.c_str()) == 0 ) {
            InterfaceXMLReader* preader = CreateInterfaceReader(_penv,itname->first,NULL,"",atts);
            if( preader->GetInterface() == NULL ) {
                RAVELOG_WARNA("failed to create interface %s in <environment>\n", itname->second.c_str());
                delete preader;
                _pcurreader.reset(new DummyXMLReader(name,"environment"));
            }
            else
                _pcurreader.reset(preader);
            return;
        }
    }

    if( stricmp((const char*)name, "environment") == 0 ) {
        _bInEnvironment = true;
    }
    else if( stricmp((const char*)name, "bkgndcolor") == 0 ||
             stricmp((const char*)name, "camrotaxis") == 0 ||
             stricmp((const char*)name, "camrotmat") == 0 ||
             stricmp((const char*)name, "camtrans") == 0 ||
             stricmp((const char*)name, "bkgndcolor") == 0 ) {
        _pcurparser = &g_realparser;
    }
    else if( stricmp((const char*)name, "plugin") == 0 ) {
        _pcurparser = &g_stringparser;
    }
    else {
        _pcurreader.reset(new DummyXMLReader(name, "environment"));
    }
}

/// if returns true, XMLReader has finished parsing
bool EnvironmentXMLReader::endElement(void *ctx ATTRIBUTE_UNUSED, const char *name)
{
    dReal* pf = _pcurparser != NULL ? (dReal*)_pcurparser->GetData() : NULL;
    
    if( !!_pcurreader ) {
        if( _pcurreader->endElement(ctx, name) ) {
            if( dynamic_cast<RobotXMLReader*>(_pcurreader.get()) ) {
                if( !_bInEnvironment )
                    ((RobotXMLReader*)_pcurreader.get())->SetXMLFilename(_filename);
                _penv->AddRobot((RobotBase*)_pcurreader->Release());
            }
            else if( dynamic_cast<KinBodyXMLReader*>(_pcurreader.get()) ) {
                if( !_bInEnvironment )
                    ((KinBodyXMLReader*)_pcurreader.get())->SetXMLFilename(_filename);
                _penv->AddKinBody((KinBody*)_pcurreader->Release());
            }
            else if( dynamic_cast<DummyInterfaceXMLReader<PT_ProblemInstance> * > (_pcurreader.get()) ) {
                _penv->LoadProblem((ProblemInstance*)_pcurreader->Release(),NULL);
            }
            else {
                //RAVELOG_WARNA("losing pointer to environment %s",name);
                _pcurreader->Release();
            }
            _pcurreader.reset();
        }
    }
    else if( stricmp((const char*)name, "bkgndcolor") == 0 ) {
        if( _pcurparser->GetCount() == 3 ) {
            vBkgndColor.x = pf[0];
            vBkgndColor.y = pf[1];
            vBkgndColor.z = pf[2];
        }
        else RAVELOG(L"bkgnd error");
    }
    else if( stricmp((const char*)name, "camrotaxis") == 0 ) {
        if( _pcurparser->GetCount() == 4 ) {
            normalize3(pf, pf);
            tCamera.rotfromaxisangle(Vector(pf[0], pf[1], pf[2]), pf[3] * PI / 180.0f);
            bTransSpecified = true;
        }
        else RAVELOG(L"camrotaxis error");
    }
    else if( stricmp((const char*)name, "camrotmat") == 0 ) {
        if( _pcurparser->GetCount() == 9 ) {
            TransformMatrix tmat;
            tmat.rotfrommat(pf[0], pf[1], pf[2], pf[3], pf[4], pf[5], pf[6], pf[7], pf[8]);
            tCamera.rot = Transform(tmat).rot;
            bTransSpecified = true;
        }
        else RAVELOG(L"camrotmat error");
    }
    else if( stricmp((const char*)name, "camtrans") == 0 ) {
        if( _pcurparser->GetCount() == 3 ) {
            tCamera.trans.x = pf[0];
            tCamera.trans.y = pf[1];
            tCamera.trans.z = pf[2];
            bTransSpecified = true;
        }
        else RAVELOG(L"camtrans error");
    }
    else if( stricmp((const char*)name, "plugin") == 0 ) {
        _penv->LoadPlugin((char*)_pcurparser->GetData());
    }
    else if( stricmp((const char*)name, "environment") == 0 ) {
        // commit changes

        //if( !bTransSpecified ) {
//            // center on first object
//            AABB ab;
//            if( g_Environ.GetRobots().size() > 0 )
//                ab = g_Environ.GetRobots().front()->ComputeAABB();
//            else if( g_Environ.GetBodies().size() > 0 )
//                ab = g_Environ.GetBodies().front()->ComputeAABB();
//            
//            TransformMatrix m(tCamera);
//            Vector vdir(m.m[2],m.m[6],m.m[10]);
//            tCamera.trans = ab.pos + 3*vdir*(fabsf(vdir.x)*ab.extents.x+fabsf(vdir.y)*ab.extents.y+fabsf(vdir.z)*ab.extents.z);
//        }

        // only move the camera if trans is specified
        if( bTransSpecified )
            _penv->SetCamera(tCamera.trans, tCamera.rot);
        
        _penv->GetViewer()->SetBkgndColor(vBkgndColor);
        return true;
    }

    _pcurparser = NULL;
    return false;
}

void EnvironmentXMLReader::characters(void *ctx ATTRIBUTE_UNUSED, const char *ch, int len)
{
    if( _pcurparser != NULL )
        _pcurparser->Format((const char*)ch, len);
    else if( !!_pcurreader )
        _pcurreader->characters(ctx, ch, len);
}
