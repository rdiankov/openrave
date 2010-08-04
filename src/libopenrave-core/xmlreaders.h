// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rosen.diankov@gmail.com)
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
#ifndef RAVE_XML_READERS
#define RAVE_XML_READERS

#include <libxml/xmlstring.h>

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

#ifdef HAVE_BOOST_FILESYSTEM
#include <boost/filesystem/operations.hpp>
#endif

BOOST_STATIC_ASSERT(sizeof(xmlChar) == 1);

#ifdef _WIN32
const char s_filesep = '\\';
#else
const char s_filesep = '/';
#endif

namespace OpenRAVEXMLParser
{
    static boost::shared_ptr<EnvironmentMutex> GetXMLMutex() {
        static boost::shared_ptr<EnvironmentMutex> m; ///< lock for parsing XML
        if( !m )
            m.reset(new EnvironmentMutex());
        return m;
    }

    /// the directory of the file currently parsing
    static string& GetParseDirectory() { static string s; return s; }
    /// full filename currently parsing
    static string& GetFullFilename() { static string s; return s; }

    static vector<string>& GetDataDirs()
    {
        static vector<string> v;
        return v;
    }

    static void SetDataDirs(const vector<string>& vdatadirs) {
        boost::shared_ptr<EnvironmentMutex> m = GetXMLMutex();
        EnvironmentMutex::scoped_lock lock(*m);
        GetDataDirs() = vdatadirs;
    }

    static int& GetXMLErrorCount()
    {
        static int errorcount=0;
        return errorcount;
    }

    typedef map<string, EnvironmentBase::CreateXMLReaderFn, CaseInsensitiveCompare> READERSMAP;
    static map<InterfaceType, READERSMAP >& GetRegisteredReaders()
    {
        static map<InterfaceType, READERSMAP > mapreaders;
        static bool s_bReadersInit = false;
        if( !s_bReadersInit ) {
            s_bReadersInit = true;
            //mapreaders[PT_KinBody][RaveGetInterfaceName(PT_KinBody)] = boost::bind(xmltag,atts
        }

        return mapreaders;
    }

    struct XMLREADERDATA
    {
        XMLREADERDATA(BaseXMLReaderPtr preader, xmlParserCtxtPtr ctxt) : _preader(preader), _ctxt(ctxt) {}
        BaseXMLReaderPtr _preader, _pdummy;
        xmlParserCtxtPtr _ctxt;
    };

    static void DefaultStartElementSAXFunc(void *ctx, const xmlChar *name, const xmlChar **atts)
    {
        std::list<std::pair<std::string,std::string> > listatts;
        if( atts != NULL ) {
            for (int i = 0;(atts[i] != NULL);i+=2) {
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

    static void DefaultEndElementSAXFunc(void *ctx, const xmlChar *name)
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

    static void DefaultCharactersSAXFunc(void *ctx, const xmlChar *ch, int len)
    {
        XMLREADERDATA* pdata = (XMLREADERDATA*)ctx;
        if( !!pdata->_pdummy )
            pdata->_pdummy->characters(string((const char*)ch, len));
        else
            pdata->_preader->characters(string((const char*)ch, len));
    }

    static void RaveXMLErrorFunc(void *ctx, const char *msg, ...)
    {
        va_list args;
        va_start(args, msg);
        RAVELOG_ERRORA("XML Parse error: ");
        vprintf(msg,args);
        va_end(args);
        GetXMLErrorCount()++;
    }

    static xmlSAXHandler* GetSAXHandler()
    {
        static xmlSAXHandler s_DefaultSAXHandler = {0};
        if( !s_DefaultSAXHandler.initialized ) {
            // first time, so init
            s_DefaultSAXHandler.startElement = DefaultStartElementSAXFunc;
            s_DefaultSAXHandler.endElement = DefaultEndElementSAXFunc;
            s_DefaultSAXHandler.characters = DefaultCharactersSAXFunc;
            s_DefaultSAXHandler.error = RaveXMLErrorFunc;
            s_DefaultSAXHandler.initialized = 1;
        }
        return &s_DefaultSAXHandler;
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

    static int raveXmlSAXUserParseFile(xmlSAXHandlerPtr sax, BaseXMLReaderPtr preader, const std::string& filename)
    {
        int ret = 0;
        xmlParserCtxtPtr ctxt;
    
        ctxt = xmlCreateFileParserCtxt(filename.c_str());
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

    static int raveXmlSAXUserParseMemory(xmlSAXHandlerPtr sax, BaseXMLReaderPtr preader, const char *buffer, int size)
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

    static boost::shared_ptr<pair<string,string> > RaveFindXMLFile(const string& filename)
    {
        if( filename.size() == 0 )
            return boost::shared_ptr<pair<string,string> >();

        boost::shared_ptr<EnvironmentMutex> m = GetXMLMutex();
        EnvironmentMutex::scoped_lock lock(*m);

        // init the dir (for some reason msvc confuses the linking of the string class with soqt, so do it the old fashioned way)
        string appended;
        size_t sepindex = filename.find_last_of('/');
        if( sepindex == string::npos )
            sepindex = filename.find_last_of('\\');

        string parsedirectory = GetParseDirectory(), fullfilename = GetFullFilename();

        // check for absolute paths
#ifdef _WIN32
        if( filename.size() > 2 && filename[1] == ':' ) {
#else
        if( filename[0] == '/' || filename[0] == '~' ) {
#endif
            parsedirectory = "";
            fullfilename = filename;
        }
        else {
            fullfilename.resize(parsedirectory.size()+filename.size());
            fullfilename = parsedirectory;
            fullfilename += filename;
        }

        if( sepindex != string::npos) {
            appended = filename.substr(0,sepindex+1);
            parsedirectory += appended;
        }

        // test if exists
        bool bFileFound = false;
        do {
            if( !!ifstream(fullfilename.c_str()) ) {
                bFileFound = true;
                break;
            }

            // try the set openrave directories
            FOREACHC(itdir, GetDataDirs()) {
                string newparse;
                newparse = *itdir; newparse.push_back(s_filesep);
                newparse += parsedirectory;
                fullfilename = newparse; fullfilename += filename;

                if( !!ifstream(fullfilename.c_str()) ) {
                    parsedirectory = newparse; parsedirectory += appended;
                    bFileFound = true;
                    break;
                }

                newparse = *itdir; newparse.push_back(s_filesep);
                fullfilename = newparse; fullfilename += filename;

                if( !!ifstream(fullfilename.c_str()) ) {
                    parsedirectory = newparse; parsedirectory += appended;
                    bFileFound = true;
                    break;
                }
            }

            if( bFileFound )
                break;

        } while(0);

        if( !bFileFound ) {
            GetXMLErrorCount()++;
            RAVELOG_WARN(str(boost::format("could not find file %s\n")%filename));
            return boost::shared_ptr<pair<string,string> >();
        }

#ifdef HAVE_BOOST_FILESYSTEM
        fullfilename = boost::filesystem::system_complete(boost::filesystem::path(fullfilename, boost::filesystem::native)).string();
#endif
        return boost::shared_ptr<pair<string,string> >(new pair<string,string>(parsedirectory,fullfilename));
    }

    static bool RaveParseXMLFile(BaseXMLReaderPtr preader, const string& filename)
    {
        boost::shared_ptr<pair<string,string> > filedata = RaveFindXMLFile(filename);
        if( !filedata )
            return false;

        boost::shared_ptr<EnvironmentMutex> m = GetXMLMutex();
        EnvironmentMutex::scoped_lock lock(*m);

        string olddir = GetParseDirectory();
        string oldfile = GetFullFilename();
        GetParseDirectory() = filedata->first;
        GetFullFilename() = filedata->second;
        preader->_filename = filedata->second;

        int ret=-1;
        try {
            ret = raveXmlSAXUserParseFile(GetSAXHandler(), preader, GetFullFilename().c_str());
            if( ret != 0 ) {
                RAVELOG_WARN(str(boost::format("xmlSAXUserParseFile: error parsing %s (error %d)\n")%GetFullFilename()%ret));
            }
        }
        catch (...) {
            RAVELOG_ERROR(str(boost::format("xmlSAXUserParseFile: error parsing %s\n")%GetFullFilename()));
            ret = -1;
        }

        // restore
        GetParseDirectory() = olddir;
        GetFullFilename() = oldfile;

        // hmm....... necessary?
        //xmlCleanupParser();
        //xmlMemoryDump();

        return ret == 0;
    }
    
    static bool RaveParseXMLData(BaseXMLReaderPtr preader, const std::string& pdata)
    {
        if( pdata.size() == 0 )
            return false;
        boost::shared_ptr<EnvironmentMutex> m = GetXMLMutex();
        EnvironmentMutex::scoped_lock lock(*m);
        return raveXmlSAXUserParseMemory(GetSAXHandler(), preader, pdata.c_str(), pdata.size())==0;
    }

    static EnvironmentBase::CreateXMLReaderFn RegisterXMLReader(InterfaceType type, const std::string& xmltag, const EnvironmentBase::CreateXMLReaderFn& fn)
    {
        boost::shared_ptr<EnvironmentMutex> m = GetXMLMutex();
        EnvironmentMutex::scoped_lock lock(*m);
        EnvironmentBase::CreateXMLReaderFn oldfn = GetRegisteredReaders()[type][xmltag];
        GetRegisteredReaders()[type][xmltag] = fn;
        return oldfn;
    }

    static void UnregisterXMLReader(InterfaceType type, const std::string& xmltag, const EnvironmentBase::CreateXMLReaderFn& oldfn)
    {
        static boost::shared_ptr<EnvironmentMutex> m = GetXMLMutex();
        EnvironmentMutex::scoped_lock lock(*m);
        GetRegisteredReaders()[type][xmltag] = oldfn;
    }

    /// mass of objects
    struct MASS
    {
    MASS() : fTotalMass(0) { for(int i = 0; i < 12; ++i) t.m[i] = 0; }
        static MASS GetBoxMass(Vector extents, Vector pos, dReal totalmass)
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
        static MASS GetBoxMassD(Vector extents, Vector pos, dReal density)
        {
            return GetBoxMass(extents, pos, 8.0f*extents.x*extents.y*extents.z*density);
        }
        static MASS GetSphericalMass(dReal radius, Vector pos, dReal totalmass)
        {
            MASS m;
            m.fTotalMass = totalmass;
            m.t = TransformMatrix();
            m.t.m[0] = m.t.m[4*1+1] = m.t.m[4*2+2]= (dReal)0.4 * totalmass * radius*radius;
            m.t.trans = pos;
            return m;
        }
        static MASS GetSphericalMassD(dReal radius, Vector pos, dReal density)
        {
            return GetSphericalMass(radius, pos, dReal(4.0)/dReal(3.0) * PI * radius * radius * radius * density);
        }
        static MASS GetCylinderMass(dReal radius, dReal height, Vector pos, dReal totalmass)
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
        static MASS GetCylinderMassD(dReal radius, dReal height, Vector pos, dReal density)
        {
            return GetCylinderMass(radius, height, pos, PI*radius*radius*height*density);
        }

        /// adds two masses together
        MASS operator+(const MASS& r) const
        {
            MASS mnew;
            if( fTotalMass+r.fTotalMass == 0 )
                MASS();
            mnew.t.trans = (fTotalMass*t.trans+r.fTotalMass*r.t.trans)*((dReal)1.0/(fTotalMass+r.fTotalMass));
            mnew.fTotalMass = fTotalMass + r.fTotalMass;
            mnew.t.m[0] = t.m[0] + r.t.m[0]; mnew.t.m[1] = t.m[1] + r.t.m[1]; mnew.t.m[2] = t.m[2] + r.t.m[2];
            mnew.t.m[4] = t.m[4] + r.t.m[4]; mnew.t.m[5] = t.m[5] + r.t.m[5]; mnew.t.m[6] = t.m[6] + r.t.m[6];
            mnew.t.m[8] = t.m[8] + r.t.m[8]; mnew.t.m[9] = t.m[9] + r.t.m[9]; mnew.t.m[10] = t.m[10] + r.t.m[10];
            return mnew;
        }

        /// adds a mass to the current mass
        MASS& operator+=(const MASS& r)
        {
            if( fTotalMass+r.fTotalMass == 0 )
                *this = MASS();
            else {
                t.trans = (fTotalMass*t.trans+r.fTotalMass*r.t.trans)*((dReal)1.0/(fTotalMass+r.fTotalMass));
                fTotalMass += r.fTotalMass;
                t.m[0] += r.t.m[0]; t.m[1] += r.t.m[1]; t.m[2] += r.t.m[2];
                t.m[4] += r.t.m[4]; t.m[5] += r.t.m[5]; t.m[6] += r.t.m[6];
                t.m[8] += r.t.m[8]; t.m[9] += r.t.m[9]; t.m[10] += r.t.m[10];
            }
            return *this;
        }

        /// transform the center of mass and inertia matrix by trans
        MASS& transform(const TransformMatrix& trans)
        {
            dReal x = trans.trans.x, y = trans.trans.y, z = trans.trans.z;
            TransformMatrix trot;
            trot = trans; trot.trans = Vector();
            t = trot * t * trot.inverse(); // rotate mass about rotation

            // translate the inertia tensor
            dReal x2 = x*x, y2 = y*y, z2 = z*z;
            t.m[0] += fTotalMass * (y2+z2); t.m[1] -= fTotalMass * x*y; t.m[2] -= fTotalMass * x * z;
            t.m[4] -= fTotalMass * y * z; t.m[5] += fTotalMass * (x2+z2); t.m[6] -= fTotalMass * y * z;
            t.m[8] -= fTotalMass * z * x; t.m[9] -= fTotalMass * z * y; t.m[10] += fTotalMass * (x2+y2);

            // ensure perfect symmetry
            t.m[5] = 0.5*(t.m[1]+t.m[5]);
            t.m[8] = 0.5*(t.m[2]+t.m[8]);
            t.m[9] = 0.5*(t.m[6]+t.m[9]);
            return *this;
        }

        TransformMatrix t; 
        dReal fTotalMass;
    };

    class KinBodyXMLReader;
    typedef boost::shared_ptr<KinBodyXMLReader> KinBodyXMLReaderPtr;
    typedef boost::shared_ptr<KinBodyXMLReader const> KinBodyXMLReaderConstPtr;
 
    class StreamXMLReader : public BaseXMLReader
    {
    public:
        virtual ProcessElement startElement(const std::string& xmlname, const std::list<std::pair<std::string,std::string> >& atts)
        {
            _ss.str(""); // have to clear the string
            if( !!_pcurreader ) {
                if( _pcurreader->startElement(xmlname,atts) == PE_Support )
                    return PE_Support;
                return PE_Ignore;
            }
            return PE_Pass;
        }

        virtual bool endElement(const std::string& xmlname)
        {
            if( !!_pcurreader ) {
                if( _pcurreader->endElement(xmlname) )
                    _pcurreader.reset();
            }
            return false;
        }
        
        virtual void characters(const std::string& ch)
        {
            if( !!_pcurreader )
                _pcurreader->characters(ch);
            else {
                _ss.clear();
                _ss << ch;
            }
        }
    protected:
        stringstream _ss;
        boost::shared_ptr<BaseXMLReader> _pcurreader;
    };

    class LinkXMLReader : public StreamXMLReader
    {
    public:
        enum MassType
        {
            MT_None = 0,
            MT_MimicGeom,
            MT_Box,
            MT_BoxMass, // use total mass instead of density
            MT_Sphere,
            MT_SphereMass,
            MT_Custom, // manually specify center of mass and inertia matrix
        };
        
  LinkXMLReader(KinBody::LinkPtr& plink, KinBodyPtr pparent, const std::list<std::pair<std::string,std::string> >& atts) : _plink(plink) {
            _pparent = pparent;
            _masstype = MT_None;
            _fMassDensity = 1;
            _vMassExtents = Vector(1,1,1);
            _fTotalMass = 1;
            _massCustom = MASS::GetSphericalMass(1,Vector(0,0,0),1);

            bool bStaticSet = false;
            bool bStatic = false;
            string linkname, linkfilename;

            FOREACHC(itatt,atts) {
                if( itatt->first == "name" ) {
                    linkname = itatt->second;
                    FOREACHC(itcurlink,pparent->GetLinks()) {
                        if( (*itcurlink)->GetName() == linkname ) {
                            _plink = *itcurlink;
                            break;
                        }
                    }
                }
                else if( itatt->first == "type" ) {
                    bStaticSet = true;
                    if( stricmp(itatt->second.c_str(), "static") == 0 ) {
                        bStatic = true;
                    }
                }
                else if( itatt->first == "file" )
                    linkfilename = itatt->second;
            }

            // if not appending to a body and plink pointer valid, append to it instead
            if( !_plink && !!plink && plink->GetParent() == pparent )
                _plink = plink;

            if( linkfilename.size() > 0 ) {
                RaveParseXMLFile(BaseXMLReaderPtr(new LinkXMLReader(_plink, _pparent, list<pair<string,string> >())), linkfilename);
            }
    
            if( !_plink )
                _plink.reset(new KinBody::Link(pparent));

            if( linkname.size() > 0 )
                _plink->name = linkname;
    
            if( bStaticSet )
                _plink->bStatic = bStatic;

            _itgeomprop = _plink->_listGeomProperties.end();
        }
        virtual ~LinkXMLReader() {}

        virtual ProcessElement startElement(const std::string& xmlname, const std::list<std::pair<std::string,std::string> >& atts)
        {
            switch( StreamXMLReader::startElement(xmlname,atts) ) {
                case PE_Pass: break;
                case PE_Support: return PE_Support;
                case PE_Ignore: return PE_Ignore;
            }

            if( _processingtag.size() > 0 ) {
                if( _processingtag == "geom" ) {
                    return (xmlname == "translation" || xmlname=="rotationmat" || xmlname=="rotationaxis" || xmlname=="quat" || xmlname=="diffusecolor" || xmlname == "ambientcolor" || xmlname == "transparency" || xmlname=="render" || xmlname == "extents" || xmlname == "radius" || xmlname == "height" || (_itgeomprop->GetType() == KinBody::Link::GEOMPROPERTIES::GeomTrimesh && (xmlname=="collision"||xmlname=="data"||xmlname=="vertices"))) ? PE_Support : PE_Ignore;
                }
                else if( _processingtag == "mass" ) {
                    return (xmlname == "density" || xmlname == "total" || xmlname == "radius" || !(_masstype == MT_Box && xmlname == "extents") || (_masstype == MT_Custom && xmlname == "com") || (_masstype == MT_Custom && xmlname == "inertia")) ? PE_Support : PE_Ignore;
                }
                return PE_Ignore;
            }

            if( xmlname == "body" ) {
                tOrigTrans = _plink->GetTransform();
                _plink->SetTransform(Transform());
                _processingtag = "";
                _pcurreader.reset(new LinkXMLReader(_plink, _pparent, atts));
                return PE_Support;
            }

            _processingtag = xmlname;
            if( xmlname == "geom" ) {
                string type;
                bool bDraw = true, bModifiable = true;
                FOREACHC(itatt,atts) {
                    if( itatt->first == "type") {
                        type = itatt->second;
                    }
                    else if( itatt->first == "render" ) {
                        // set draw to false only if atts[i]==false
                        bDraw = stricmp(itatt->second.c_str(), "false")!=0;
                    }
                    else if( itatt->first == "modifiable" )
                        bModifiable = !(stricmp(itatt->second.c_str(), "false") == 0 || itatt->second=="0");
                }

                if( type.size() == 0 ) {
                    RAVELOG_INFOA("no geometry type, defaulting to box\n");
                    type = "box";
                }

                _itgeomprop = _plink->_listGeomProperties.insert(_plink->_listGeomProperties.end(),KinBody::Link::GEOMPROPERTIES(_plink));
                if( stricmp(type.c_str(), "box") == 0 )
                    _itgeomprop->type = KinBody::Link::GEOMPROPERTIES::GeomBox;
                else if( stricmp(type.c_str(), "sphere") == 0 )
                    _itgeomprop->type = KinBody::Link::GEOMPROPERTIES::GeomSphere;
                else if( stricmp(type.c_str(), "cylinder") == 0 )
                    _itgeomprop->type = KinBody::Link::GEOMPROPERTIES::GeomCylinder;
                else if( stricmp(type.c_str(), "trimesh") == 0 ) {
                    _itgeomprop->type = KinBody::Link::GEOMPROPERTIES::GeomTrimesh;
                }
                else {
                    RAVELOG_WARNA(str(boost::format("type %s not supported\n")%type));
                }

                _itgeomprop->_bDraw = bDraw;
                _itgeomprop->_bModifiable = bModifiable;
                return PE_Support;
            }
            else if( xmlname == "translation" || xmlname == "rotationmat" || xmlname == "rotationaxis" || xmlname == "quat" || xmlname == "offsetfrom" ) {
                return PE_Support;
            }
            else if( xmlname == "mass" ) {
                // find the type of mass and create
                _masstype = MT_Sphere;
                FOREACHC(itatt,atts) {
                    if( itatt->first == "type") {
                        if( stricmp(itatt->second.c_str(), "mimicgeom") == 0 ) {
                            _masstype = MT_MimicGeom;
                        }
                        else if( stricmp(itatt->second.c_str(), "box") == 0 ) {
                            _masstype = MT_Box;
                        }
                        else if( stricmp(itatt->second.c_str(), "sphere") == 0 ) {
                            _masstype = MT_Sphere;
                        }
                        else if( stricmp(itatt->second.c_str(), "custom") == 0 ) {
                            _masstype = MT_Custom;
                        }
                        break;
                    }
                }
                return PE_Support;
            }

            _processingtag = "";
            return PE_Pass;
        }

        virtual bool endElement(const std::string& xmlname)
        {
            if( !!_pcurreader ) {
                if( _pcurreader->endElement(xmlname) ) {
                    if( xmlname == "body" ) {
                        // directly apply transform to all geomteries
                        Transform tnew = _plink->GetTransform();
                        FOREACH(itgeomprop, _plink->_listGeomProperties)
                            itgeomprop->_t = tnew * itgeomprop->_t;
                        _plink->collision.ApplyTransform(tnew);
                        _plink->SetTransform(tOrigTrans);
                    }

                    _pcurreader.reset();
                }
                return false;
            }
            else if( _processingtag == "geom" ) {
                if( xmlname == "translation" ) {
                    Vector v;
                    _ss >>v .x >> v.y >> v.z;
                    _itgeomprop->_t.trans += v;
                }
                else if( xmlname == "rotationmat" ) {
                    TransformMatrix tnew;
                    _ss >> tnew.m[0] >> tnew.m[1] >> tnew.m[2] >> tnew.m[4] >> tnew.m[5] >> tnew.m[6] >> tnew.m[8] >> tnew.m[9] >> tnew.m[10];
                    _itgeomprop->_t.rot = (Transform(tnew)*_itgeomprop->_t).rot;
                }
                else if( xmlname == "rotationaxis" ) {
                    Vector vaxis; dReal fangle=0;
                    _ss >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
                    Transform tnew; tnew.rotfromaxisangle(vaxis.normalize3(), fangle * PI / 180.0f);
                    _itgeomprop->_t.rot = (tnew*_itgeomprop->_t).rot;
                }
                else if( xmlname == "quat" ) {
                    Transform tnew;
                    _ss >> tnew.rot.x >> tnew.rot.y >> tnew.rot.z >> tnew.rot.w;
                    tnew.rot.normalize4();
                    _itgeomprop->_t.rot = (tnew*_itgeomprop->_t).rot;
                }
                else if( xmlname == "render" ) {
                    // check attributes for format (default is vrml)
                    string renderfile;
                    _itgeomprop->vRenderScale = Vector(1,1,1);
                    _ss >> renderfile;
                    _ss >> _itgeomprop->vRenderScale.x; _itgeomprop->vRenderScale.y = _itgeomprop->vRenderScale.z = _itgeomprop->vRenderScale.x;
                    _ss >> _itgeomprop->vRenderScale.y >> _itgeomprop->vRenderScale.z;
                    _itgeomprop->renderfile = !_fnGetModelsDir ? renderfile : _fnGetModelsDir(renderfile);
                }
                else if( xmlname == "diffusecolor" ) {
                    _ss >> _itgeomprop->diffuseColor.x >> _itgeomprop->diffuseColor.y >> _itgeomprop->diffuseColor.z;
                }
                else if( xmlname == "ambientcolor" ) {
                    _ss >> _itgeomprop->ambientColor.x >> _itgeomprop->ambientColor.y >> _itgeomprop->ambientColor.z;
                }
                else if( xmlname == "transparency" ) {
                    _ss >> _itgeomprop->ftransparency;
                }
                else if( xmlname == _processingtag ) {
                    if( _itgeomprop->type == KinBody::Link::GEOMPROPERTIES::GeomCylinder ) { // axis has to point on y
                        // rotate on x axis by pi/2
                        Transform trot;
                        trot.rotfromaxisangle(Vector(1, 0, 0), PI/2);
                        _itgeomprop->_t.rot = (_itgeomprop->_t*trot).rot;
                    }

                    // call before attaching the geom
                    if( _itgeomprop->type != KinBody::Link::GEOMPROPERTIES::GeomTrimesh ) {
                        _itgeomprop->InitCollisionMesh();
                    }

                    _plink->collision.Append(_itgeomprop->GetCollisionMesh(), _itgeomprop->_t);
                    _itgeomprop = _plink->_listGeomProperties.end();
                    _processingtag = "";
                }
                else {
                    // could be type specific features
                    switch(_itgeomprop->GetType()) {
                    case KinBody::Link::GEOMPROPERTIES::GeomSphere:
                        if( xmlname == "radius" )
                            _ss >> _itgeomprop->vGeomData.x;
                        break;
                    case KinBody::Link::GEOMPROPERTIES::GeomBox:
                        if( xmlname == "extents" )
                            _ss >> _itgeomprop->vGeomData.x >> _itgeomprop->vGeomData.y >> _itgeomprop->vGeomData.z;
                        break;
                    case KinBody::Link::GEOMPROPERTIES::GeomCylinder:
                        if( xmlname == "radius")
                            _ss >> _itgeomprop->vGeomData.x;
                        else if( xmlname == "height" )
                            _ss >> _itgeomprop->vGeomData.y;
                        break;
                    case KinBody::Link::GEOMPROPERTIES::GeomTrimesh:
                        if( xmlname == "data" || xmlname == "collision") {
                            string orgrenderfile;
                            Vector vScale(1,1,1);
                            _ss >> orgrenderfile;
                            _ss >> vScale.x; vScale.y = vScale.z = vScale.x;
                            _ss >> vScale.y >> vScale.z;
                            string renderfile = !_fnGetModelsDir ? orgrenderfile : _fnGetModelsDir(orgrenderfile);

                            bool bSuccess = false;
                            if( renderfile.size() > 0 ) {
#ifdef OPENRAVE_COIN3D
                                SoDB::readlock(); // have to lock coin3d, or otherwise state gets corrupted
                                SoInput mySceneInput;
                                if (!mySceneInput.openFile(renderfile.c_str())) {
                                    RAVELOG_WARNA(str(boost::format("Failed to open %s for KinBody:TriMesh\n")%renderfile));
                                    GetXMLErrorCount()++;
                                }
                                else {
                                    // SoDB::readAll memory leaks!
                                    SoSeparator* psep = SoDB::readAll(&mySceneInput);
                                    if( !!psep ) {
                                        CreateTriMeshData(psep, _itgeomprop->collisionmesh);
                                        psep->unref();
                                        
                                        //_trimeshGeom
                                        FOREACH(it, _itgeomprop->collisionmesh.vertices)
                                            *it *= vScale;
                                        bSuccess = true;
                                    }
                                }
                        
                                mySceneInput.closeFile();
                                SoDB::readunlock();
#endif

                                if( !bSuccess ) {
                                    ivcon::ReadFile(renderfile.c_str(), _itgeomprop->collisionmesh);
                                    RAVELOG_VERBOSEA(str(boost::format("trimesh verts: %d, inds: %d\n")%_itgeomprop->collisionmesh.vertices.size()%_itgeomprop->collisionmesh.indices.size()));
                                    FOREACH(it, _itgeomprop->collisionmesh.vertices)
                                        *it *= vScale;
                                }
                            }
                            else
                                RAVELOG_WARNA(str(boost::format("failed to find %s\n")%orgrenderfile));
                        }
                        else if( xmlname == "vertices" ) {
                            vector<dReal> values((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
                            if( (values.size()%9) ) {
                                RAVELOG_WARN(str(boost::format("number of points specified in the vertices field of %s body needs to be a multiple of 3 (it is %d), ignoring...\n")%_plink->GetName()%values.size()));
                            }
                            else {
                                _itgeomprop->collisionmesh.vertices.resize(values.size()/3);
                                _itgeomprop->collisionmesh.indices.resize(values.size()/3);
                                vector<dReal>::iterator itvalue = values.begin();
                                size_t i = 0;
                                FOREACH(itv,_itgeomprop->collisionmesh.vertices) {
                                    itv->x = *itvalue++;
                                    itv->y = *itvalue++;
                                    itv->z = *itvalue++;
                                    _itgeomprop->collisionmesh.indices[i] = i;
                                    ++i;
                                }
                            }
                        }
                        break;
                    default:
                        _pcurreader.reset(new DummyXMLReader(xmlname,"geom"));
                    }
                }
                return false;
            }
            if( _processingtag == "mass" ) {
                if( xmlname == "density" ) {
                    if( _masstype == MT_BoxMass )
                        _masstype = MT_Box;
                    else if( _masstype == MT_SphereMass)
                        _masstype = MT_Sphere;
                    _ss >> _fMassDensity;
                }
                else if( xmlname == "total" ) {
                    if( _masstype == MT_Box )
                        _masstype = MT_BoxMass;
                    else if( _masstype == MT_Sphere)
                        _masstype = MT_SphereMass;
                    _ss >> _fTotalMass;
                    _massCustom.fTotalMass = _fTotalMass;
                }
                else if( xmlname == "radius" ) {
                    _ss >> _vMassExtents.x;
                }
                else if( _masstype == MT_Box && xmlname == "extents" ) {
                    _ss >> _vMassExtents.x >> _vMassExtents.y >> _vMassExtents.z;
                }
                else if( xmlname == _processingtag ) {
                    _processingtag = "";
                }
                else if( _masstype == MT_Custom ) {
                    if( xmlname == "com" ) {
                        _ss >> _plink->_transMass.trans.x >> _plink->_transMass.trans.y >> _plink->_transMass.trans.z;
                        _massCustom.t.trans = _plink->_transMass.trans;
                    }
                    else if( xmlname == "inertia" ) {
                        _ss >> _massCustom.t.m[0] >> _massCustom.t.m[1] >> _massCustom.t.m[2] >> _massCustom.t.m[4] >> _massCustom.t.m[5] >> _massCustom.t.m[6] >> _massCustom.t.m[8] >> _massCustom.t.m[9] >> _massCustom.t.m[10];
                    }
                }
                return false;
            }

            if( xmlname == "body" ) {
                if( _plink->GetGeometries().size() == 0 )
                    RAVELOG_WARNA(str(boost::format("link %s has no geometry attached!\n")%_plink->GetName()));

                // perform final processing stages
                MASS totalmass;
                if( _masstype == MT_MimicGeom ) {
                    FOREACHC(itgeom, _plink->GetGeometries()) {
                        MASS mass;
                        switch(itgeom->type) {
                        case KinBody::Link::GEOMPROPERTIES::GeomSphere:
                            mass = MASS::GetSphericalMassD(itgeom->GetSphereRadius(), Vector(),_fMassDensity);
                            break;
                        case KinBody::Link::GEOMPROPERTIES::GeomBox:
                            mass = MASS::GetBoxMassD(itgeom->GetBoxExtents(), Vector(), _fMassDensity);
                            break;
                        case KinBody::Link::GEOMPROPERTIES::GeomCylinder:
                            mass = MASS::GetCylinderMassD(itgeom->GetCylinderRadius(), itgeom->GetCylinderHeight(), Vector(), _fMassDensity);
                            break;
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
                    totalmass = MASS::GetSphericalMass(_vMassExtents.x, Vector(), _fTotalMass);
        
                _plink->_transMass = totalmass.t;
                _plink->_mass = totalmass.fTotalMass;
                tOrigTrans = _plink->GetTransform();

                Transform cur;
                if( !!_offsetfrom ) {
                    // recompute new transformation
                    Transform root;
                    if( !!_fnGetOffsetFrom )
                        root = _fnGetOffsetFrom(_offsetfrom);
                    else
                        root = _offsetfrom->GetTransform();

                    cur = _plink->GetTransform();
                    cur = root * cur;
                    tOrigTrans = root * tOrigTrans; // update orig trans separately
                    _plink->SetTransform(cur);
                }

                return true;
            }

            if( xmlname == "translation" ) {
                Vector v;
                _ss >> v.x >> v.y >> v.z;
                _plink->_t.trans += v;
            }
            else if( xmlname == "rotationmat" ) {
                TransformMatrix tnew;
                _ss >> tnew.m[0] >> tnew.m[1] >> tnew.m[2] >> tnew.m[4] >> tnew.m[5] >> tnew.m[6] >> tnew.m[8] >> tnew.m[9] >> tnew.m[10];
                _plink->_t.rot = (Transform(tnew)*_plink->_t).rot;
            }
            else if( xmlname == "rotationaxis" ) {
                Vector vaxis; dReal fangle=0;
                _ss >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
                Transform tnew; tnew.rotfromaxisangle(vaxis.normalize3(), fangle * PI / 180.0f);
                _plink->_t.rot = (tnew*_plink->_t).rot;
            }
            else if( xmlname == "quat" ) {
                Transform tnew;
                _ss >> tnew.rot.x >> tnew.rot.y >> tnew.rot.z >> tnew.rot.w;
                tnew.rot.normalize4();
                _plink->_t.rot = (tnew*_plink->_t).rot;
            }
            else if( xmlname == "offsetfrom" ) {
                // figure out which body
                string linkname;
                _ss >> linkname;
                _offsetfrom = _pparent->GetLink(linkname);

                if( !_offsetfrom ) {
                    RAVELOG_WARNA(str(boost::format("Failed to find offsetfrom body %s\n")%linkname));
                    GetXMLErrorCount()++;
                }
            }
            
            if( xmlname !=_processingtag )
                RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%xmlname%_processingtag));
            _processingtag = "";
            return false;
        }

        void SetMassType(MassType type, float fValue, const Vector& vMassExtents)
        {
            _masstype = type;
            _fMassDensity = _fTotalMass = fValue;
            _vMassExtents = vMassExtents;
        }

        Transform GetOrigTransform() const { return tOrigTrans; }

        boost::function<string(const std::string&)> _fnGetModelsDir;
        boost::function<Transform(KinBody::LinkPtr)> _fnGetOffsetFrom;

    private:
        MASS _mass;                    ///< current mass of the object
        KinBody::LinkPtr& _plink;
        KinBodyPtr _pparent;
        KinBody::LinkPtr _offsetfrom;                    ///< all transformations are relative to the this body
        list<KinBody::Link::GEOMPROPERTIES>::iterator _itgeomprop;

        Transform tOrigTrans;
        // Mass
        MassType _masstype;           ///< if true, mass is craeted so that it mimics the geometry
        string _processingtag; /// if not empty, currently processing
        MASS _massCustom;
        float _fMassDensity, _fTotalMass;
        Vector _vMassExtents;           ///< used only if mass is a box
    };

    // Joint Reader
    class JointXMLReader : public StreamXMLReader
    {
    public:
    JointXMLReader(KinBody::JointPtr& pjoint, KinBodyPtr pparent, const std::list<std::pair<std::string,std::string> >& atts) : _pjoint(pjoint) {
            _bNegateJoint = false;
            _bMimicJoint = false;
            bDisabled = false;
            _pparent = pparent;
            _pjoint.reset(new KinBody::Joint(pparent));
            _pjoint->type = KinBody::Joint::JointHinge;

            FOREACHC(itatt,atts) {
                if( itatt->first == "name" ) {
                    _pjoint->name = itatt->second;
                }
                else if( itatt->first == "type" ) {
                    if( stricmp(itatt->second.c_str(), "hinge") == 0 )
                        _pjoint->type = KinBody::Joint::JointHinge;
                    else if( stricmp(itatt->second.c_str(), "slider") == 0 )
                        _pjoint->type = KinBody::Joint::JointSlider;
                    else if( stricmp(itatt->second.c_str(), "universal") == 0 )
                        _pjoint->type = KinBody::Joint::JointUniversal;
                    else if( stricmp(itatt->second.c_str(), "hinge2") == 0 )
                        _pjoint->type = KinBody::Joint::JointHinge2;
                    else if( stricmp(itatt->second.c_str(), "spherical") == 0 ) {
                        _pjoint->type = KinBody::Joint::JointSpherical;
                        _pjoint->vAxes[0] = Vector(1,0,0);
                        _pjoint->vAxes[1] = Vector(0,1,0);
                        _pjoint->vAxes[2] = Vector(0,0,1);
                    }
                    else {
                        RAVELOG_WARNA(str(boost::format("unrecognized joint type: %s, setting to hinge\n")%itatt->second));
                        _pjoint->type = KinBody::Joint::JointHinge;
                    }
                }
                else if( itatt->first == "enable" ) {
                    bDisabled = stricmp(itatt->second.c_str(), "false") == 0 || itatt->second=="0";
                }
                else if( itatt->first == "mimic" ) {
                    stringstream ss(itatt->second);
                    ss >> _strmimicjoint >> _pjoint->vMimicCoeffs[0] >> _pjoint->vMimicCoeffs[1];
                    if( !ss ) {
                        RAVELOG_WARNA(str(boost::format("failed to set mimic properties correctly from: %s\n")%itatt->second));
                    }
                    _bMimicJoint = true;
                }
                else if( itatt->first == "circular" )
                    _pjoint->_bIsCircular = !(stricmp(itatt->second.c_str(), "false") == 0 || itatt->second=="0");
            }

            _pjoint->_vlowerlimit.resize(_pjoint->GetDOF());
            _pjoint->_vupperlimit.resize(_pjoint->GetDOF());
            if( _pjoint->GetType() == KinBody::Joint::JointSlider ) {
                for(int i = 0; i < _pjoint->GetDOF(); ++i) {
                    _pjoint->_vlowerlimit[i] = -100000;
                    _pjoint->_vupperlimit[i] = 100000;
                }
            }
            else if( _pjoint->GetType() == KinBody::Joint::JointSpherical ) {
                for(int i = 0; i < _pjoint->GetDOF(); ++i) {
                    _pjoint->_vlowerlimit[i] = -1000;
                    _pjoint->_vupperlimit[i] = 1000;
                }
            }
            else {
                for(int i = 0; i < _pjoint->GetDOF(); ++i) {
                    _pjoint->_vlowerlimit[i] = -PI;
                    _pjoint->_vupperlimit[i] = PI;
                }
            }

            _pjoint->_vweights.resize(_pjoint->GetDOF());
            FOREACH(it,_pjoint->_vweights)
                *it = 1;
        }

        bool IsDisabled() const { return bDisabled; }
        bool IsMimic() const { return _bMimicJoint; }
        const string& GetMimicJoint() const { return _strmimicjoint; }

        virtual ProcessElement startElement(const std::string& xmlname, const std::list<std::pair<std::string,std::string> >& atts)
        {
            switch( StreamXMLReader::startElement(xmlname,atts) ) {
                case PE_Pass: break;
                case PE_Support: return PE_Support;
                case PE_Ignore: return PE_Ignore;
            }
            
            return (xmlname == "body" || xmlname == "offsetfrom" || xmlname == "weight" || xmlname == "lostop" || xmlname == "histop" || xmlname == "limits" || xmlname == "limitsrad" || xmlname == "limitsdeg" || xmlname == "maxvel" || xmlname == "hardmaxvel" || xmlname == "maxaccel" || xmlname == "maxtorque" || xmlname == "maxforce" || xmlname == "resolution" || xmlname == "anchor" || xmlname == "axis" || xmlname == "axis1" || xmlname == "axis2" || xmlname=="axis3" || xmlname == "mode") ? PE_Support : PE_Pass;
        }

        virtual bool endElement(const std::string& xmlname)
        {
            int numindices = _pjoint->GetDOF();
            dReal fRatio = _pjoint->type == KinBody::Joint::JointSlider ? (dReal)1 : (dReal)PI / 180.0f; // most, but not all, joint take degrees
        
            if( !!_pcurreader ) {
                if( _pcurreader->endElement(xmlname) )
                    _pcurreader.reset();
            }
            else if( xmlname == "joint" ) {
                if( _pparent->GetLinks().size() == 0 ) {
                    RAVELOG_WARNA("parent kinbody has no links defined yet!\n");
                    return false;
                }
            
                string defaultname = "J_";
            
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

                if( !_pjoint->bodies[0] || !_pjoint->bodies[1] ) {
                    RAVELOG_WARNA(str(boost::format("one or more attached bodies are invalid for joint %s\n")%_pjoint->GetName()));
                    if( !_pjoint->bodies[1] )
                        _pjoint->bodies[1] = _pparent->GetLinks().at(0);
                    if( !_pjoint->bodies[0] )
                        _pjoint->bodies[0] = _pparent->GetLinks().at(0);
                }

                // make sure first body is always closer to the root, unless the second body is static
                if( !_pjoint->bodies[1]->IsStatic() ) {
                    if( _pjoint->bodies[0]->IsStatic() || (_pjoint->bodies[0]->GetIndex() > _pjoint->bodies[1]->GetIndex() && !_pjoint->bodies[1]->IsStatic()) ) {
                        for(int i = 0; i < _pjoint->GetDOF(); ++i) {
                            _pjoint->vAxes[i] = -_pjoint->vAxes[i];
                        }
                        swap(_pjoint->bodies[0], _pjoint->bodies[1]);
                    }
                }

                if( _bNegateJoint ) {
                    for(int i = 0; i < _pjoint->GetDOF(); ++i) {
                        _pjoint->vAxes[i] = -_pjoint->vAxes[i];
                    }
                }

                tbody0 = _pjoint->bodies[0]->GetTransform();
                tbody1 = _pjoint->bodies[1]->GetTransform();

                Transform toffsetfrom;
                if( !!_offsetfrom ) {
                    if( !!_fnGetOffsetFrom ) {
                        toffsetfrom = _fnGetOffsetFrom(_offsetfrom);
                    }
                    else {
                        toffsetfrom = _offsetfrom->GetTransform();
                    }
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
                    RAVELOG_WARNA(str(boost::format("joint %s: all attached links are static!\n")%_pjoint->GetName()));
                }

                _pjoint->vanchor = toffsetfrom*_pjoint->vanchor;
                for(int i = 0; i < _pjoint->GetDOF(); ++i) {
                    _pjoint->vAxes[i] = toffsetfrom.rotate(_pjoint->vAxes[i]);
                }

                switch(_pjoint->type) {
                case KinBody::Joint::JointHinge:
                    _pjoint->tLeft.rotfromaxisangle(_pjoint->vAxes[0], _pjoint->offset);
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
                case KinBody::Joint::JointSpherical:
                    _pjoint->tLeft.trans = _pjoint->vanchor;
                    _pjoint->tRight.trans = -_pjoint->vanchor;
                    _pjoint->tRight = _pjoint->tRight * trel;
                    break;
                default:
                    throw openrave_exception(str(boost::format("unknown joint type %d")%_pjoint->type));
                }

                if( _pjoint->bodies[1]->IsStatic() ) {
                    _pjoint->tLeft = _pparent->GetTransform().inverse() * tbody1 * _pjoint->tLeft;
                    _pjoint->bodies[0]->SetTransform(_pjoint->tLeft * _pjoint->tRight);
                }
                else
                    _pjoint->bodies[1]->SetTransform(tbody0 * _pjoint->tLeft * _pjoint->tRight);

                _pjoint->tinvRight = _pjoint->tRight.inverse();
                _pjoint->tinvLeft = _pjoint->tLeft.inverse();

                // have to transform back
                if( !!_pjoint->bodies[0] )
                    _pjoint->bodies[0]->SetTransform(tbody0);
                if( !!_pjoint->bodies[1] )
                    _pjoint->bodies[1]->SetTransform(tbody1);

                return true;
            }
            else if( xmlname == "weight" ) {
                for(int i = 0; i < numindices; ++i)
                    _ss >> _pjoint->_vweights.at(i);
            }
            else if( xmlname == "body" ) {
                // figure out which body
                int index = !_pjoint->bodies[0] ? 0 : 1;
                bool bQuery = true;
                string linkname;
                _ss >> linkname;

                FOREACHC(itlink, _pparent->GetLinks()) {
                    if( stricmp((*itlink)->GetName().c_str(), linkname.c_str()) == 0 ) {
                        bQuery = !(*itlink)->IsStatic();
                        _pjoint->bodies[index] = *itlink;
                        break;
                    }
                }
            
                if( !_pjoint->bodies[index] && bQuery ) {
                    RAVELOG_WARNA(str(boost::format("Failed to find body %s for joint %s\n")%linkname%_pjoint->name));
                    GetXMLErrorCount()++;
                }
            }
            else if( xmlname == "limits" || xmlname == "limitsrad" || xmlname == "limitsdeg" ) {
                if( _bNegateJoint ) {
                    throw openrave_exception("cannot specify <limits> with <lostop> and <histop>, choose one");
                }
                dReal fmult = xmlname == "limitsdeg" ? fRatio : dReal(1.0);
                vector<dReal> values = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
                if( (int)values.size() == 2*_pjoint->GetDOF() ) {
                    for(int i = 0; i < _pjoint->GetDOF(); ++i ) {
                        _pjoint->_vlowerlimit.at(i) = fmult*min(values[2*i+0],values[2*i+1]);
                        _pjoint->_vupperlimit.at(i) = fmult*max(values[2*i+0],values[2*i+1]);
                    }
                }
                else {
                    RAVELOG_WARN(str(boost::format("<limits> tag has %d values, expected %d! ignoring...\n")%values.size()%_pjoint->GetDOF()));
                }
            }
            else if( xmlname == "lostop" ) {
                _bNegateJoint = true;
                RAVELOG_ERROR(str(boost::format("%s: <lostop> is deprecated, please use <limits> (now in radians), <limitsrad>, or <limitsdeg> tag and negate your joint axis!\n")%_pparent->GetName()));
                _pjoint->_vlowerlimit.resize(numindices);
                FOREACH(it,_pjoint->_vlowerlimit) {
                    _ss >> *it;
                    *it *= fRatio;
                }
            }
            else if( xmlname == "histop" ) {
                _bNegateJoint = true;
                RAVELOG_ERROR(str(boost::format("%s: <histop> deprecated, please use <limits> (now in radians), <limitsrad>, <limitsdeg> tag and negate your joint axis!\n")%_pparent->GetName()));
                _pjoint->_vupperlimit.resize(numindices);
                FOREACH(it,_pjoint->_vupperlimit) {
                    _ss >> *it;
                    *it *= fRatio;
                }
            }
            else if( xmlname == "maxvel" )
                _ss >> _pjoint->fMaxVel;
            else if( xmlname == "hardmaxvel" )
                _ss >> _pjoint->fHardMaxVel;
            else if( xmlname == "maxaccel" )
                _ss >> _pjoint->fMaxAccel;
            else if( xmlname == "maxtorque" )
                _ss >> _pjoint->fMaxTorque;
            else if( xmlname == "resolution" ) {
                _ss >> _pjoint->fResolution;
                _pjoint->fResolution *= fRatio;
            }
            else if( xmlname == "offsetfrom" ) {
                // figure out which body
                string linkname; _ss >> linkname;
                _offsetfrom = _pparent->GetLink(linkname);
        
                if( !_offsetfrom ) {
                    RAVELOG_WARNA(str(boost::format("Failed to find body %s\n")%linkname));
                    GetXMLErrorCount()++;
                }
            }
            else {
                // could be type specific
                switch(_pjoint->type) {
                case KinBody::Joint::JointHinge:
                    if( xmlname == "anchor" )
                        _ss >> _pjoint->vanchor.x >> _pjoint->vanchor.y >> _pjoint->vanchor.z;
                    else if( xmlname == "axis" ) {
                        _ss >> _pjoint->vAxes[0].x >> _pjoint->vAxes[0].y >> _pjoint->vAxes[0].z;
                        _pjoint->vAxes[0].normalize3();
                    }
                    break;
                case KinBody::Joint::JointSlider:
                    if( xmlname == "axis" ) {
                        _ss >> _pjoint->vAxes[0].x >> _pjoint->vAxes[0].y >> _pjoint->vAxes[0].z;
                        _pjoint->vAxes[0].normalize3();
                    }
                    break;
                case KinBody::Joint::JointUniversal:
                    if( xmlname == "anchor" )
                        _ss >> _pjoint->vanchor.x >> _pjoint->vanchor.y >> _pjoint->vanchor.z;
                    else if( xmlname == "axis1" ) {
                        _ss >> _pjoint->vAxes[0].x >> _pjoint->vAxes[0].y >> _pjoint->vAxes[0].z;
                        _pjoint->vAxes[0].normalize3();
                    }
                    else if( xmlname == "axis2" ) {
                        _ss >> _pjoint->vAxes[1].x >> _pjoint->vAxes[1].y >> _pjoint->vAxes[1].z;
                        _pjoint->vAxes[0].normalize3();
                    }
                    break;
                case KinBody::Joint::JointHinge2:
                    if( xmlname == "anchor" )
                        _ss >> _pjoint->vanchor.x >> _pjoint->vanchor.y >> _pjoint->vanchor.z;
                    else if( xmlname == "axis1" ) {
                        _ss >> _pjoint->vAxes[0].x >> _pjoint->vAxes[0].y >> _pjoint->vAxes[0].z;
                        _pjoint->vAxes[0].normalize3();
                    }
                    else if( xmlname == "axis2" ) {
                        _ss >> _pjoint->vAxes[1].x >> _pjoint->vAxes[1].y >> _pjoint->vAxes[1].z;
                        _pjoint->vAxes[0].normalize3();
                    }
                    break;
                case KinBody::Joint::JointSpherical:
                    if( xmlname == "anchor" )
                        _ss >> _pjoint->vanchor.x >> _pjoint->vanchor.y >> _pjoint->vanchor.z;
                    break;
                default:
                    throw openrave_exception(str(boost::format("bad joint type: %d")%_pjoint->type));
                    break;
                }
            }

            return false;
        }

        boost::function<string(const std::string&)> _fnGetModelsDir;
        boost::function<Transform(KinBody::LinkPtr)> _fnGetOffsetFrom;

    private:
        KinBody::LinkPtr _offsetfrom; ///< all transforms are relative to this body
        KinBodyPtr _pparent;
        KinBody::JointPtr& _pjoint;
        string _strmimicjoint;
        bool bDisabled; // if true, joint is not counted as a controllable degree of freedom
        bool _bMimicJoint;
        bool _bNegateJoint;
    };

    class InterfaceXMLReader;
    typedef boost::shared_ptr<InterfaceXMLReader> InterfaceXMLReaderPtr;
    typedef boost::shared_ptr<InterfaceXMLReader const> InterfaceXMLReaderConstPtr;

    static InterfaceXMLReaderPtr CreateInterfaceReader(EnvironmentBasePtr penv, InterfaceType type, InterfaceBasePtr& pinterface, const std::string& xmltag, const std::list<std::pair<std::string,std::string> >& atts);

    class InterfaceXMLReader : public StreamXMLReader
    {
    public:
    InterfaceXMLReader(EnvironmentBasePtr penv, InterfaceBasePtr& pinterface, InterfaceType type, const string& xmltag, const std::list<std::pair<std::string,std::string> >& atts) : _penv(penv), _type(type), _pinterface(pinterface), _xmltag(xmltag) {
            string strtype;
            FOREACHC(itatt,atts) {
                if( itatt->first == "type" ) {
                    strtype = itatt->second;
                }
                else if( itatt->first == "file" ) {
                    std::list<std::pair<std::string,std::string> > listnewatts;
                    FOREACHC(itatt2,atts) {
                        if( itatt2->first != "file" )
                            listnewatts.push_back(*itatt2);
                    }

                    //BaseXMLReaderPtr preader = CreateInterfaceReader(_penv,_type,_pinterface, xmltag, listnewatts);
                    //bool bSuccess = RaveParseXMLFile(preader, itatt->second);
                    boost::shared_ptr<pair<string,string> > filedata = RaveFindXMLFile(itatt->second);
                    if( !filedata ) {
                        continue;
                    }

                    string olddir = GetParseDirectory();
                    string oldfile = GetFullFilename();
                    try {
                        GetParseDirectory() = filedata->first;
                        GetFullFilename() = filedata->second;
                        pinterface = _penv->ReadInterfaceXMLFile(pinterface,_type,filedata->second,listnewatts);
                    }
                    catch(...) {
                        RAVELOG_ERROR(str(boost::format("failed to process %s\n")%itatt->second));
                        pinterface.reset();
                    }
                    GetParseDirectory() = olddir;
                    GetFullFilename() = oldfile;
                    
                    if( !pinterface ) {
                        RAVELOG_DEBUGA(str(boost::format("Failed to load kinbody filename %s\n")%itatt->second));
                        GetXMLErrorCount()++;
                        break;
                    }
                    _filename = pinterface->GetXMLFilename();
                }
            }

            if( !_pinterface ) {
                if( strtype.size() > 0 ) {
                    _pinterface = _penv->CreateInterface(type,strtype);
                    if( !_pinterface )
                        GetXMLErrorCount()++;
                }
        
                if( !_pinterface ) {
                    switch(type) {
                    case PT_KinBody:
                        _pinterface = penv->CreateKinBody();
                        break;
                    case PT_Robot:
                        _pinterface = _penv->CreateInterface(PT_Robot, "GenericRobot");
                        if( !_pinterface )
                            _pinterface = _penv->CreateInterface(PT_Robot, "");
                        break;
                    case PT_Controller:
                        _pinterface = _penv->CreateInterface(PT_Controller, "IdealController");
                        break;
                    default:
                        _pinterface = _penv->CreateInterface(type, "");
                        break;
                    }
                }
            }

            if( !_pinterface )
                RAVELOG_ERRORA(str(boost::format("xml readers failed to create instance of type %s:%s\n")%RaveGetInterfaceName(type)%strtype));
            else {
                _type = _pinterface->GetInterfaceType();

                // check to see if a reader is registered for this type
                READERSMAP::iterator it = GetRegisteredReaders()[_type].find(_pinterface->GetXMLId());
                if( it != GetRegisteredReaders()[_type].end() )
                    _pcustomreader = it->second(_pinterface, atts);
            }

            if( _xmltag.size() == 0 )
                _xmltag = RaveGetInterfaceName(_type);

            SetXMLFilename(_filename);
        }

        void SetXMLFilename(const string& filename) { if( !!_pinterface && _pinterface->__strxmlfilename.size() == 0 ) _pinterface->__strxmlfilename = filename; }

        virtual ProcessElement startElement(const std::string& xmlname, const std::list<std::pair<std::string,std::string> >& atts)
        {
            if( !_pinterface )
                return PE_Ignore;

            if( !!_pcustomreader ) {
                if( _pcustomreader->startElement(xmlname, atts) == PE_Support )
                    return PE_Support;
                return PE_Ignore;
            }
            
            switch( StreamXMLReader::startElement(xmlname,atts) ) {
                case PE_Pass: break;
                case PE_Support: return PE_Support;
                case PE_Ignore: return PE_Ignore;
            }

            // check for registers readers
            READERSMAP::iterator it = GetRegisteredReaders()[_type].find(xmlname);
            if( it != GetRegisteredReaders()[_type].end() ) {
                _readername = xmlname;
                _pcustomreader = it->second(_pinterface, atts);
                if( !!_pcustomreader )
                    return PE_Support;
            }

            if (xmlname == "sendcommand" ) {
                return PE_Support;
            }
            
            return PE_Pass;
        }

        virtual bool endElement(const std::string& xmlname)
        {
            if( !!_pcurreader ) {
                if( _pcurreader->endElement(xmlname) )
                    _pcurreader.reset();
            }
            else if( !!_pcustomreader ) {
                if( _pcustomreader->endElement(xmlname) ) {
                    if( _readername.size() > 0 )
                        _pinterface->__mapReadableInterfaces[_readername] = _pcustomreader->GetReadable();
                    _pcustomreader.reset();

                    if( xmlname == _xmltag )
                        return true;
                }
            }
            else if( xmlname == "sendcommand" ) {
                if( !!_pinterface ) {
                    stringstream sout;
                    if( !_pinterface->SendCommand(sout,_ss) ) {
                        RAVELOG_WARN("interface command failed\n");
                    }
                }
                else {
                    RAVELOG_INFO(str(boost::format("failed to send command: %s\n")%xmlname));
                }
            }
            else if( xmlname == _xmltag )
                return true;
            return false;
        }

        virtual void characters(const std::string& ch)
        {
            if( !!_pcustomreader )
                _pcustomreader->characters(ch);
            else
                StreamXMLReader::characters(ch);
        }

        InterfaceBasePtr GetInterface() { return _pinterface; }

    protected:        
        EnvironmentBasePtr _penv;
        InterfaceType _type;
        InterfaceBasePtr& _pinterface;
        BaseXMLReaderPtr _pcustomreader;
        string _xmltag;
        string _interfacename, _readername;
    };
    /// KinBody reader
    /// reads kinematic chain specific entries, can instantiate this reader from another reader
    class KinBodyXMLReader : public InterfaceXMLReader
    {
    public:
    KinBodyXMLReader(EnvironmentBasePtr penv, InterfaceBasePtr& pchain, InterfaceType type, const std::list<std::pair<std::string,std::string> >& atts, int rootoffset, int rootjoffset) : InterfaceXMLReader(penv,pchain,type,"kinbody",atts), rootoffset(rootoffset), rootjoffset(rootjoffset) {
            _pchain = RaveInterfaceCast<KinBody>(_pinterface);
            _masstype = LinkXMLReader::MT_None;
            _fMassValue = 1;
            _vMassExtents = Vector(1,1,1);
            _bOverwriteDiffuse = false;
            _bOverwriteAmbient = false;
            _bOverwriteTransparency = false;

            FOREACHC(itatt,atts) {
                if( itatt->first == "prefix" ) {
                    _prefix = itatt->second;
                }
                else if( itatt->first == "name" ) {
                    _bodyname = itatt->second;
                }
                else if( itatt->first == "makejoinedlinksadjacent")
                    _pchain->_bMakeJoinedLinksAdjacent = atoi(itatt->second.c_str())!=0;
                else if( itatt->first != "file" && itatt->first != "type" )
                    RAVELOG_WARN(str(boost::format("unknown kinbody attribute %s\n")%itatt->first));
            }

            // reisze _vTransforms to be the same size as the initial number of links
            _pchain->GetBodyTransformations(_vTransforms);
            _pchain->SetGuiData(boost::shared_ptr<void>());
        }

        Transform GetOffsetFrom(KinBody::LinkPtr plink)
        {
            if( plink->GetIndex() < 0 || plink->GetIndex() >= (int)_vTransforms.size() )
                return plink->GetTransform();
            return _vTransforms[plink->GetIndex()];
        }
        
        string GetModelsDir(const std::string& filename) const
        {
            if( filename.size() == 0 )
                return filename;
#ifdef _WIN32
            if( filename.find_first_of(':') != string::npos )
                return filename;
#else
            if( filename[0] == '/' || filename[0] == '~' )
                return filename;
#endif

            list<string> listmodelsdir;
            listmodelsdir.push_back(_strModelsDir);
            if( GetParseDirectory().size() > 0 ) {
                listmodelsdir.push_back(GetParseDirectory());
                listmodelsdir.back().push_back(s_filesep);
                listmodelsdir.back() += _strModelsDir;
            }

            string temp;
            FOREACH(itmodelsdir, listmodelsdir) {
                temp = *itmodelsdir; temp += filename;
                //RAVELOG_INFOA("modelsdir: %s, modelname: %s\n", itmodelsdir->c_str(), temp.c_str());
                if( !!ifstream(temp.c_str()) )
                    return temp;

                FOREACHC(itdir, GetDataDirs()) {
                    temp = *itdir; temp.push_back(s_filesep);
                    temp += *itmodelsdir;
                    temp += filename;
                    if( !!ifstream(temp.c_str()) )
                        return temp;
                }
            }

            return ""; // bad filename
        }
    
        virtual ProcessElement startElement(const std::string& xmlname, const std::list<std::pair<std::string,std::string> >& atts)
        {
            if( _processingtag.size() > 0 ) {
                switch( StreamXMLReader::startElement(xmlname,atts) ) {
                    case PE_Pass: break;
                    case PE_Support: return PE_Support;
                    case PE_Ignore: return PE_Ignore;
                }
            
                if( _processingtag == "mass" ) {
                    return (xmlname == "density" || xmlname == "total" || xmlname == "radius" || (_masstype == LinkXMLReader::MT_Box && xmlname == "extents") || (_masstype == LinkXMLReader::MT_Custom && (xmlname == "com"||xmlname == "inertia"))) ? PE_Support : PE_Ignore;
                }
                return PE_Ignore;
            }

            switch( InterfaceXMLReader::startElement(xmlname,atts) ) {
                case PE_Pass: break;
                case PE_Support: return PE_Support;
                case PE_Ignore: return PE_Ignore;
            }

            if( xmlname == "kinbody" ) {
                _pcurreader = CreateInterfaceReader(_penv,PT_KinBody,_pinterface, xmlname, atts);
                return PE_Support;
            }
            else if( xmlname == "body" ) {
                _plink.reset();
                boost::shared_ptr<LinkXMLReader> plinkreader(new LinkXMLReader(_plink, _pchain, atts));
                plinkreader->SetMassType(_masstype, _fMassValue, _vMassExtents);
                plinkreader->_fnGetModelsDir = boost::bind(&KinBodyXMLReader::GetModelsDir,this,_1);
                plinkreader->_fnGetOffsetFrom = boost::bind(&KinBodyXMLReader::GetOffsetFrom,this,_1);
                _pcurreader = plinkreader;
                return PE_Support;
            }
            else if( xmlname == "joint" ) {
                _pjoint.reset();
                _pchain->_vecJointIndices.push_back(_pchain->GetDOF()); // be careful, GetDOF() is computed using _vecJointIndices
                boost::shared_ptr<JointXMLReader> pjointreader(new  JointXMLReader(_pjoint,_pchain, atts));
                pjointreader->_fnGetModelsDir = boost::bind(&KinBodyXMLReader::GetModelsDir,this,_1);
                pjointreader->_fnGetOffsetFrom = boost::bind(&KinBodyXMLReader::GetOffsetFrom,this,_1);
                _pcurreader = pjointreader;
                return PE_Support;
            }

            if( xmlname == "mass" ) {
                // find the type of mass and create
                _masstype = LinkXMLReader::MT_Sphere;
                FOREACHC(itatt, atts) {
                    if( itatt->first == "type" ) {
                        if( stricmp(itatt->second.c_str(), "mimicgeom") == 0 ) {
                            _masstype = LinkXMLReader::MT_MimicGeom;
                        }
                        else if( stricmp(itatt->second.c_str(), "box") == 0 ) {
                            _masstype = LinkXMLReader::MT_Box;
                        }
                        else if( stricmp(itatt->second.c_str(), "sphere") == 0 ) {
                            _masstype = LinkXMLReader::MT_Sphere;
                        }
                
                        break;
                    }
                }

                _processingtag = xmlname;
                return PE_Support;
            }

            if (xmlname == "translation" || xmlname == "rotationmat" || xmlname == "rotationaxis" || xmlname == "quat" || xmlname == "jointvalues" || xmlname == "adjacent" || xmlname == "modelsdir" || xmlname == "diffusecolor" || xmlname == "transparency" || xmlname == "ambientcolor" ) {
                _processingtag = xmlname;
                return PE_Support;
            }
            return PE_Pass;
        }
        
        virtual bool endElement(const std::string& xmlname)
        {
            if( !!_pcurreader ) {
                if( _pcurreader->endElement(xmlname) ) {
                    if( xmlname == "body" ) {
                        if( !_plink )
                            throw openrave_exception("link should be valid");

                        if( _plink->index < 0 ) {
                            // not in array yet
                            _plink->index = (int)_pchain->_veclinks.size();
                            _pchain->_veclinks.push_back(_plink);
                            _vTransforms.push_back(Transform());
                        }

                        // do this later, or else offsetfrom will be messed up!
                        _vTransforms[_plink->GetIndex()] = boost::dynamic_pointer_cast<LinkXMLReader>(_pcurreader)->GetOrigTransform();
                        _plink.reset();
                    }
                    else if( xmlname == "joint" ) {
                        if( !_pjoint )
                            throw openrave_exception("joint should be valid");
                        BOOST_ASSERT(_pchain->_vecJointIndices.size()>0);
                        _pjoint->dofindex = (int)_pchain->_vecJointIndices.back();
                        boost::shared_ptr<JointXMLReader> pjointreader = boost::dynamic_pointer_cast<JointXMLReader>(_pcurreader);
                        if( pjointreader->IsMimic() )
                            listMimicJoints.push_back(make_pair(_pjoint,pjointreader->GetMimicJoint()));

                        if( pjointreader->IsDisabled() ) {
                            _pchain->_vecJointIndices.pop_back();
                            _pjoint->jointindex = -1;
                            _pjoint->dofindex = -1;
                            _pchain->_vecPassiveJoints.push_back(_pjoint);
                        }
                        else {
                            _pjoint->jointindex = (int)_pchain->_vecjoints.size();
                            _pchain->_vecjoints.push_back(_pjoint);
                        }
                        BOOST_ASSERT( _pjoint->dofindex < _pchain->GetDOF());
                        _pjoint.reset();
                    }
                    else if( xmlname == "kinbody" ) {
                        // most likely new transforms were added, so update
                        _pchain = RaveInterfaceCast<KinBody>(_pinterface);
                        _pchain->GetBodyTransformations(_vTransforms);
                    }
                    else
                        RAVELOG_INFOA(str(boost::format("releasing unknown tag %s\n")%xmlname));

                    _pcurreader.reset();
                }
            }
            else if( _processingtag == "mass" ) {
                if( xmlname == "mass" ) {
                    _processingtag = "";
                }
                else if( xmlname == "density" ) {
                    if( _masstype == LinkXMLReader::MT_BoxMass )
                        _masstype = LinkXMLReader::MT_Box;
                    else if( _masstype == LinkXMLReader::MT_SphereMass)
                        _masstype = LinkXMLReader::MT_Sphere;
                    _ss >> _fMassValue;
                }
                else if( xmlname == "total" ) {
                    if( _masstype == LinkXMLReader::MT_Box )
                        _masstype = LinkXMLReader::MT_BoxMass;
                    else if( _masstype == LinkXMLReader::MT_Sphere)
                        _masstype = LinkXMLReader::MT_SphereMass;
                    _ss >> _fMassValue;
                }
                else if( xmlname == "radius" ) {
                    _ss >> _vMassExtents.x;
                }
                else if( _masstype == LinkXMLReader::MT_Box && xmlname == "extents" ) {
                    _ss >> _vMassExtents.x >> _vMassExtents.y >> _vMassExtents.z;
                }
            }
            else if( _processingtag.size() > 0 ) {
                if( xmlname == "translation" ) {
                    Vector v;
                    _ss >> v.x >> v.y >> v.z;
                    _trans.trans += v;
                }
                else if( xmlname == "rotationaxis" ) {
                    Vector vaxis; dReal fangle=0;
                    _ss >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
                    Transform tnew; tnew.rotfromaxisangle(vaxis.normalize3(), fangle * PI / 180.0f);
                    _trans.rot = (tnew*_trans).rot;
                }
                else if( xmlname == "quat" ) {
                    Transform tnew;
                    _ss >> tnew.rot.x >> tnew.rot.y >> tnew.rot.z >> tnew.rot.w;
                    tnew.rot.normalize4();
                    _trans.rot = (tnew*_trans).rot;
                }
                else if( xmlname == "rotationmat" ) {
                    TransformMatrix tnew;
                    _ss >> tnew.m[0] >> tnew.m[1] >> tnew.m[2] >> tnew.m[4] >> tnew.m[5] >> tnew.m[6] >> tnew.m[8] >> tnew.m[9] >> tnew.m[10];
                    _trans.rot = (Transform(tnew)*_trans).rot;
                }
                else if( xmlname == "adjacent" ) {
                    pair<string, string> entry;
                    _ss >> entry.first >> entry.second;
                    _pchain->_vForcedAdjacentLinks.push_back(entry);
                }
                else if( xmlname == "modelsdir" ) {
                    _ss >> _strModelsDir;
                    _strModelsDir += "/";
                }
                else if( xmlname == "diffuseColor" ) {
                    // check attributes for format (default is vrml)
                    _ss >> _diffusecol.x >> _diffusecol.y >> _diffusecol.z;
                    _bOverwriteDiffuse = true;
                }
                else if( xmlname == "ambientColor" ) {
                    // check attributes for format (default is vrml)
                    _ss >> _ambientcol.x >> _ambientcol.y >> _ambientcol.z;
                    _bOverwriteAmbient = true;
                }
                else if( xmlname == "transparency" ) {
                    _ss >> _transparency;
                    _bOverwriteTransparency = true;
                }
                else if( xmlname == "jointvalues" ) {
                    _vjointvalues = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
                }

                if( xmlname !=_processingtag )
                    RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%xmlname%_processingtag));
                _processingtag = "";
            }
            else if( InterfaceXMLReader::endElement(xmlname) ) {
                // go through all mimic joints and assign the correct indices
                FOREACH(itmimic, listMimicJoints) {
                    itmimic->first->nMimicJointIndex = _pchain->GetJointIndex(itmimic->second);
                    if( itmimic->first->nMimicJointIndex < 0 ) {
                        RAVELOG_WARNA("Failed to find mimic joint: %s", itmimic->second.c_str());
                        GetXMLErrorCount()++;
                    }
                }
                
                if( _bodyname.size() > 0 )
                    _pchain->SetName(_bodyname);
                if( _filename.size() > 0 ) {
                    SetXMLFilename(_filename);
                }

                // add prefix
                if( _prefix.size() > 0 ) {
                    for(vector<KinBody::LinkPtr>::iterator itlink = _pchain->_veclinks.begin()+rootoffset; itlink != _pchain->_veclinks.end(); ++itlink)
                        (*itlink)->name = _prefix + (*itlink)->name;
                    for(vector<KinBody::JointPtr>::iterator itjoint = _pchain->_vecjoints.begin()+rootjoffset; itjoint != _pchain->_vecjoints.end(); ++itjoint)
                        (*itjoint)->name = _prefix +(*itjoint)->name;
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

                for(vector<KinBody::LinkPtr>::iterator itlink = _pchain->_veclinks.begin()+rootoffset; itlink != _pchain->_veclinks.end(); ++itlink)
                    (*itlink)->SetTransform(_trans * (*itlink)->GetTransform());

                if( _vjointvalues.size() > 0 ) {
                    if( _vjointvalues.size() == _pchain->GetJoints().size() )
                        _pchain->SetJointValues(_vjointvalues);
                    else
                        RAVELOG_WARNA(str(boost::format("jointvalues for body %s wrong number (%d!=%d)\n")%_pchain->GetName()%_vjointvalues.size()%_pchain->GetJoints().size()));
                }
        
                Vector com = _pchain->GetCenterOfMass();
                RAVELOG_VERBOSEA("%s: COM = (%f,%f,%f)\n", _pchain->GetName().c_str(), com.x, com.y, com.z);
                return true;
            }
            return false;
        }

    protected:
        KinBodyPtr _pchain;
        Transform _trans;

        // default mass type passed to every LinkXMLReader
        int rootoffset, rootjoffset;                 ///< the initial number of links when KinBody is created (so that global translations and rotations only affect the new links)
        LinkXMLReader::MassType _masstype;             ///< if true, mass is craeted so that it mimics the geometry
        float _fMassValue;               ///< density or total mass
        Vector _vMassExtents;

        vector<Transform> _vTransforms;     ///< original transforms of the bodies for offsetfrom

        string _strModelsDir, _bodyname;
        string _prefix; ///< add this prefix to all names of links and joints
        KinBody::LinkPtr _plink;
        KinBody::JointPtr _pjoint;

        RaveVector<float> _diffusecol, _ambientcol;
        float _transparency;
        vector<dReal> _vjointvalues;

        string _processingtag; /// if not empty, currently processing
        bool _bOverwriteDiffuse, _bOverwriteAmbient, _bOverwriteTransparency;

        list<pair<KinBody::JointPtr,string> > listMimicJoints; ///< mimic joints needed to be resolved
    };

    class ControllerXMLReader : public InterfaceXMLReader
    {
    public:
        ControllerXMLReader(EnvironmentBasePtr penv, InterfaceBasePtr& pinterface, const std::list<std::pair<std::string,std::string> >& atts,RobotBasePtr probot=RobotBasePtr()) : InterfaceXMLReader(penv,pinterface,PT_Controller,RaveGetInterfaceName(PT_Controller),atts) {
            _probot = probot;
            FOREACHC(itatt, atts) {
                if( itatt->first == "robot" )
                    _robotname = itatt->second;
                else if( itatt->first == "args" )
                    _args = itatt->second;
            }
        }
        virtual ~ControllerXMLReader() {}

        virtual bool endElement(const std::string& xmlname)
        {
            if( InterfaceXMLReader::endElement(xmlname) ) {
                if( !_probot ) {
                    if( _robotname.size() > 0 ) {
                        KinBodyPtr pbody = _penv->GetKinBody(_robotname.c_str());
                        if( pbody->IsRobot() )
                            _probot = RaveInterfaceCast<RobotBase>(pbody);
                    }
                }
                
                if( !!_probot )
                    _probot->SetController(RaveInterfaceCast<ControllerBase>(_pinterface),_args);
                else
                    RAVELOG_WARNA("controller is unused\n");
                return true;
            }
            return false;
        }

        string _robotname, _args;
        RobotBasePtr _probot;
    };

    class ManipulatorXMLReader : public StreamXMLReader
    {
    public:
    ManipulatorXMLReader(RobotBase::ManipulatorPtr& pmanip, RobotBasePtr probot, const std::list<std::pair<std::string,std::string> >& atts) : _pmanip(pmanip) {
            if( !_pmanip )
                _pmanip.reset(new RobotBase::Manipulator(probot));

            FOREACHC(itatt,atts) {
                if( itatt->first == "name" ) {
                    _pmanip->_name = itatt->second;
                }
            }

            _probot = _pmanip->GetRobot();
        }

        virtual ProcessElement startElement(const std::string& xmlname, const std::list<std::pair<std::string,std::string> >& atts)
        {
            switch( StreamXMLReader::startElement(xmlname,atts) ) {
                case PE_Pass: break;
                case PE_Support: return PE_Support;
                case PE_Ignore: return PE_Ignore;
            }

            if( _processingtag.size() > 0 )
                return PE_Ignore;

            if (xmlname == "effector" || xmlname == "gripperjoints" || xmlname == "joints" || xmlname == "armjoints" || xmlname == "base"|| xmlname == "iksolver" || xmlname == "closingdir" || xmlname == "palmdirection" || xmlname=="direction" || xmlname == "closingdirection" || xmlname == "translation" || xmlname == "quat" || xmlname == "rotationaxis" || xmlname == "rotationmat") {
                _processingtag = xmlname;
                return PE_Support;
            }
            return PE_Pass;
        }

        virtual bool endElement(const std::string& xmlname)
        {
            if( StreamXMLReader::endElement(xmlname) )
                return true;

            if( xmlname == "manipulator" ) {
                if( _pmanip->_vClosingDirection.size() == 0 ) {
                    RAVELOG_DEBUG(str(boost::format("setting manipulator %s closing direction to zeros\n")%_pmanip->GetName()));
                    _pmanip->_vClosingDirection.resize(_pmanip->_vgripperdofindices.size(),0);
                }
                else if( _pmanip->_vgripperdofindices.size() != _pmanip->_vClosingDirection.size() ) {
                    RAVELOG_WARN(str(boost::format("Manipulator %s has closing direction grasps wrong %d!=%d\n")%_pmanip->GetName()%_pmanip->_vgripperdofindices.size()%_pmanip->_vClosingDirection.size()));
                    _pmanip->_vClosingDirection.resize(_pmanip->_vgripperdofindices.size(),0);
                }
                return true;
            }
            else if( xmlname == "effector" ) {
                // look up the correct link
                string linkname; _ss >> linkname;
                _pmanip->_pEndEffector = _probot->GetLink(linkname);
        
                if( !_pmanip->_pEndEffector ) {
                    RAVELOG_WARNA("Failed to find manipulator end effector %s\n", linkname.c_str());
                    GetXMLErrorCount()++;
                }
            }
            else if( xmlname == "base" ) {
                string linkname; _ss >> linkname;
                _pmanip->_pBase = _probot->GetLink(linkname);
                if( !_pmanip->_pBase ) {
                    RAVELOG_WARNA("Failed to find manipulator base %s\n", linkname.c_str());
                    GetXMLErrorCount()++;
                }
            }
            else if( xmlname == "joints" || xmlname == "gripperjoints" ) {
                vector<string> jointnames((istream_iterator<string>(_ss)), istream_iterator<string>());
                _pmanip->_vgripperdofindices.resize(0);
                FOREACH(itname,jointnames) {
                    int index = _probot->GetJointIndex(*itname);
                    if( index < 0 )
                        RAVELOG_WARNA(str(boost::format("failed to find gripper joint name %s\n")%*itname));
                    else
                        _pmanip->_vgripperdofindices.push_back(index);
                }
            }
            else if( xmlname == "armjoints" ) {
                RAVELOG_WARN("<armjoints> for <manipulator> tag is not used anymore\n");
            }
            else if( xmlname == "direction" || xmlname == "palmdirection" ) {
                if( xmlname == "palmdirection" )
                    RAVELOG_WARN("<palmdirection> tag in Manipulator changed to <direction>\n");
                _ss >> _pmanip->_vdirection.x >> _pmanip->_vdirection.y >> _pmanip->_vdirection.z;
                dReal flen = _pmanip->_vdirection.lengthsqr3();
                if( flen == 0 ) {
                    RAVELOG_WARNA("palm direction is 0, setting to default value\n");
                    _pmanip->_vdirection = Vector(0,0,1);
                }
                else
                    _pmanip->_vdirection /= RaveSqrt(flen);
            }
            else if( xmlname == "iksolver" ) {
                string iklibraryname = _ss.str();

                IkSolverBasePtr piksolver;
                if( _probot->GetEnv()->HasInterface(PT_InverseKinematicsSolver,iklibraryname) )
                    piksolver = _probot->GetEnv()->CreateIkSolver(iklibraryname);
                if( !piksolver ) {
                    // try adding the current directory
                    if( _probot->GetEnv()->HasInterface(PT_InverseKinematicsSolver,GetParseDirectory()+s_filesep+iklibraryname)) {
                        string fullname = GetParseDirectory(); fullname.push_back(s_filesep); fullname += iklibraryname;
                        piksolver = _probot->GetEnv()->CreateIkSolver(fullname);
                    }

                    if( !piksolver ) {
                        // try loading the shared object
                        ProblemInstancePtr pIKFastLoader;
                        {
                            list<ProblemInstancePtr> listProblems;
                            boost::shared_ptr<void> pmutex = _probot->GetEnv()->GetLoadedProblems(listProblems);
                            FOREACHC(itprob, listProblems) {
                                if( stricmp((*itprob)->GetXMLId().c_str(),"ikfast") == 0 ) {
                                    pIKFastLoader = *itprob;
                                    break;
                                }
                            }
                        }

                        if( !pIKFastLoader ) {
                            pIKFastLoader = _probot->GetEnv()->CreateProblem("ikfast");
                            if( !!pIKFastLoader )
                                _probot->GetEnv()->LoadProblem(pIKFastLoader,"");
                        }

                        if( !!pIKFastLoader ) {
                            string ikonly;
                            _ss >> ikonly;
                            stringstream scmd(string("AddIkLibrary ") + ikonly + string(" ") + ikonly);
                            stringstream sout;
                            if( !ifstream(ikonly.c_str()) || !pIKFastLoader->SendCommand(sout, scmd)) {
                                string fullname = GetParseDirectory(); fullname.push_back(s_filesep); fullname += ikonly;
                                scmd.str(string("AddIkLibrary ") + fullname + string(" ") + fullname);
                                if( !ifstream(fullname.c_str()) || !pIKFastLoader->SendCommand(sout, scmd)) {
                                }
                                else {
                                    // need to use the original iklibrary string due to parameters being passed in
                                    string fullname = "ikfast ";
                                    fullname += GetParseDirectory(); fullname.push_back(s_filesep); fullname += iklibraryname;
                                    piksolver = _probot->GetEnv()->CreateIkSolver(fullname);
                                }
                            }
                            else {
                                string fullname = "ikfast "; fullname += iklibraryname;
                                piksolver = _probot->GetEnv()->CreateIkSolver(fullname);
                            }
                        }
                        else {
                            RAVELOG_WARNA("Failed to load IKFast problem\n");
                        }
                    }
                }
                
                if( !piksolver )
                    RAVELOG_WARNA("failed to create iksolver %s\n", iklibraryname.c_str());
                else {
                    _pmanip->_strIkSolver = piksolver->GetXMLId();
                }

                _pmanip->_pIkSolver = piksolver;
                if( !!piksolver ) {
                    _pmanip->_strIkSolver = piksolver->GetXMLId();
                }
            }
            else if( xmlname == "closingdirection" || xmlname == "closingdir" ) {
                _pmanip->_vClosingDirection = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
                FOREACH(it, _pmanip->_vClosingDirection) {
                    if( *it > 0 )
                        *it = 1;
                    else if( *it < 0 )
                        *it = -1;
                }
            }
            else if( xmlname == "translation" ) {
                Vector v;
                _ss >> v.x >> v.y >> v.z;
                _pmanip->_tGrasp.trans += v;
            }
            else if( xmlname == "quat" ) {
                Transform tnew;
                _ss >> tnew.rot.x >> tnew.rot.y >> tnew.rot.z >> tnew.rot.w;
                tnew.rot.normalize4();
                _pmanip->_tGrasp.rot = (tnew*_pmanip->_tGrasp).rot;
            }
            else if( xmlname == "rotationaxis" ) {
                Vector vaxis; dReal fangle=0;
                _ss >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
                Transform tnew; tnew.rotfromaxisangle(vaxis.normalize3(), fangle * PI / 180.0f);
                _pmanip->_tGrasp.rot = (tnew*_pmanip->_tGrasp).rot;
            }
            else if( xmlname == "rotationmat" ) {
                TransformMatrix tnew;
                _ss >> tnew.m[0] >> tnew.m[1] >> tnew.m[2] >> tnew.m[4] >> tnew.m[5] >> tnew.m[6] >> tnew.m[8] >> tnew.m[9] >> tnew.m[10];
                _pmanip->_tGrasp.rot = (Transform(tnew)*_pmanip->_tGrasp).rot;
            }

            if( xmlname !=_processingtag )
                RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%xmlname%_processingtag));
            _processingtag = "";
            return false;
        }
    
    protected:
        RobotBasePtr _probot;
        RobotBase::ManipulatorPtr& _pmanip;
        string _processingtag;
    };

    /// sensors specifically attached to a robot
    class AttachedSensorXMLReader : public StreamXMLReader
    {
    public:
    AttachedSensorXMLReader(RobotBase::AttachedSensorPtr& psensor, RobotBasePtr probot, const std::list<std::pair<std::string,std::string> >& atts) : _psensor(psensor) {
            string name;
            FOREACHC(itatt, atts) {
                if( itatt->first == "name" ) {
                    name = itatt->second;
                }
            }
            
            if( !_psensor ) {
                // check for current sensors
                FOREACH(itsensor,probot->GetAttachedSensors()) {
                    if( name.size() > 0 && (*itsensor)->GetName() == name ) {
                        _psensor = *itsensor;
                        break;
                    }
                }

                if( !_psensor ) {
                    _psensor.reset(new RobotBase::AttachedSensor(probot));
                    probot->_vecSensors.push_back(_psensor);
                }
            }

            _psensor->_name = name;
            _probot = _psensor->GetRobot();
        }
        
        virtual ProcessElement startElement(const std::string& xmlname, const std::list<std::pair<std::string,std::string> >& atts)
        {
            switch( StreamXMLReader::startElement(xmlname,atts) ) {
                case PE_Pass: break;
                case PE_Support: return PE_Support;
                case PE_Ignore: return PE_Ignore;
            }

            if( _processingtag.size() > 0 )
                return PE_Ignore;

            if( xmlname == "sensor" ) {
                // create the sensor
                _psensorinterface.reset();
                _pcurreader = CreateInterfaceReader(_probot->GetEnv(),PT_Sensor,_psensorinterface, xmlname, atts);
                return PE_Support;
            }
            
            if (xmlname == "link" || xmlname == "translation" || xmlname == "quat" || xmlname == "rotationaxis" || xmlname == "rotationmat" ) {
                _processingtag = xmlname;
                return PE_Support;
            }
            return PE_Pass;
        }
        virtual bool endElement(const std::string& xmlname)
        {
            if( !!_pcurreader ) {
                if( _pcurreader->endElement(xmlname) ) {
                    _pcurreader.reset();
                    _psensor->psensor = RaveInterfaceCast<SensorBase>(_psensorinterface);
                }
                return false;
            }
            else if( xmlname == "attachedsensor" ) {
                if( !_psensor->psensor ) {
                    RAVELOG_VERBOSEA("Attached robot sensor %s points to no real sensor!\n",_psensor->GetName().c_str());
                }
                else {
                    if( !_psensor->psensor->Init(args) ) {
                        RAVELOG_WARNA("failed to initialize sensor %s\n", _psensor->GetName().c_str());
                        _psensor->psensor.reset();
                    }
                    else {
                        _psensor->pdata = _psensor->psensor->CreateSensorData();
                        if( _psensor->pattachedlink.expired() ) {
                            RAVELOG_INFOA("no attached link, setting to base of robot\n");
                            if( _probot->GetLinks().size() == 0 ) {
                                RAVELOG_INFOA("robot has no links!\n");
                                _psensor->pattachedlink.reset();
                            }
                            else
                                _psensor->pattachedlink = _probot->GetLinks().at(0);
                        }
                    }
                }

                return true;
            }
            else if( xmlname == "link" ) {
                string linkname;
                _ss >> linkname;
                _psensor->pattachedlink = _probot->GetLink(linkname);
        
                if( _psensor->pattachedlink.expired() ) {
                    RAVELOG_WARNA("Failed to find attached sensor link %s\n", linkname.c_str());
                    GetXMLErrorCount()++;
                }
            }
            else if( xmlname == "translation" ) {
                Vector v;
                _ss >> v.x >> v.y >> v.z;
                _psensor->trelative.trans += v;
            }
            else if( xmlname == "quat" ) {
                Transform tnew;
                _ss >> tnew.rot.x >> tnew.rot.y >> tnew.rot.z >> tnew.rot.w;
                tnew.rot.normalize4();
                _psensor->trelative.rot = (tnew*_psensor->trelative).rot;
            }
            else if( xmlname == "rotationaxis" ) {
                Vector vaxis; dReal fangle=0;
                _ss >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
                Transform tnew; tnew.rotfromaxisangle(vaxis.normalize3(), fangle * PI / 180.0f);
                _psensor->trelative.rot = (tnew*_psensor->trelative).rot;
            }
            else if( xmlname == "rotationmat" ) {
                TransformMatrix tnew;
                _ss >> tnew.m[0] >> tnew.m[1] >> tnew.m[2] >> tnew.m[4] >> tnew.m[5] >> tnew.m[6] >> tnew.m[8] >> tnew.m[9] >> tnew.m[10];
                _psensor->trelative.rot = (Transform(tnew)*_psensor->trelative).rot;
            }

            if( xmlname !=_processingtag )
                RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%xmlname%_processingtag));
            _processingtag = "";
            return false;
        }
        
    protected:
        RobotBasePtr _probot;
        RobotBase::AttachedSensorPtr& _psensor;
        InterfaceBasePtr _psensorinterface;
        string _processingtag;
        string args; ///< arguments to pass to sensor when initializing
    };

    class RobotXMLReader : public InterfaceXMLReader
    {
    public:
    RobotXMLReader(EnvironmentBasePtr penv, InterfaceBasePtr& probot, const std::list<std::pair<std::string,std::string> >& atts, int rootoffset, int rootjoffset, int rootsoffset, int rootmoffset) : InterfaceXMLReader(penv,probot,PT_Robot,"robot",atts), rootoffset(rootoffset), rootjoffset(rootjoffset), rootsoffset(rootsoffset), rootmoffset(rootmoffset) {
            _probot = RaveInterfaceCast<RobotBase>(_pinterface);
            FOREACHC(itatt, atts) {
                if( itatt->first == "name" )
                    _robotname = itatt->second;
                else if( itatt->first == "prefix" )
                    _prefix = itatt->second;
            }
        }

        virtual ProcessElement startElement(const std::string& xmlname, const std::list<std::pair<std::string,std::string> >& atts)
        {
            if( _processingtag.size() > 0 ) {
                switch( StreamXMLReader::startElement(xmlname,atts) ) {
                    case PE_Pass: break;
                    case PE_Support: return PE_Support;
                    case PE_Ignore: return PE_Ignore;
                }
                return PE_Ignore;
            }
            
            switch( InterfaceXMLReader::startElement(xmlname,atts) ) {
                case PE_Pass: break;
                case PE_Support: return PE_Support;
                case PE_Ignore: return PE_Ignore;
            }

            if( xmlname == "robot" ) {
                _pcurreader = CreateInterfaceReader(_penv, PT_Robot, _pinterface, xmlname, atts);
            }
            else if( xmlname == "kinbody" ) {
                _pcurreader = CreateInterfaceReader(_penv,PT_KinBody,_pinterface, xmlname, atts);
            }
            else if( xmlname == "manipulator" ) {
                _probot->_vecManipulators.push_back(RobotBase::ManipulatorPtr(new RobotBase::Manipulator(_probot)));
                _pcurreader.reset(new ManipulatorXMLReader(_probot->_vecManipulators.back(), _probot, atts));
            }
            else if( xmlname == "attachedsensor" ) {
                _psensor.reset();
                _pcurreader.reset(new AttachedSensorXMLReader(_psensor, _probot, atts));
            }
            else if( xmlname == "controller" ) {
                _pcontroller.reset();
                _pcurreader.reset(new ControllerXMLReader(_probot->GetEnv(),_pcontroller,atts,_probot));
            }
            else if( xmlname == "translation" || xmlname == "rotationmat" || xmlname == "rotationaxis" || xmlname == "quat" || xmlname == "jointvalues") {
                _processingtag = xmlname;
            }
            else
                return PE_Pass;
            return PE_Support;
        }
        virtual bool endElement(const std::string& xmlname)
        {
            if( !!_pcurreader ) {
                if( _pcurreader->endElement(xmlname) )
                    _pcurreader.reset();
                _probot = RaveInterfaceCast<RobotBase>(_pinterface); // might be updated by readers
                return false;
            }
            else if( _processingtag.size() > 0 ) {
                if( xmlname == "translation" ) {
                    Vector v;
                    _ss >> v.x >> v.y >> v.z;
                    _trans.trans += v;
                }
                else if( xmlname == "rotationaxis" ) {
                    Vector vaxis; dReal fangle=0;
                    _ss >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
                    Transform tnew; tnew.rotfromaxisangle(vaxis.normalize3(), fangle * PI / 180.0f);
                    _trans.rot = (tnew*_trans).rot;
                }
                else if( xmlname == "quat" ) {
                    Transform tnew;
                    _ss >> tnew.rot.x >> tnew.rot.y >> tnew.rot.z >> tnew.rot.w;
                    tnew.rot.normalize4();
                    _trans.rot = (tnew*_trans).rot;
                }
                else if( xmlname == "rotationmat" ) {
                    TransformMatrix tnew;
                    _ss >> tnew.m[0] >> tnew.m[1] >> tnew.m[2] >> tnew.m[4] >> tnew.m[5] >> tnew.m[6] >> tnew.m[8] >> tnew.m[9] >> tnew.m[10];
                    _trans.rot = (Transform(tnew)*_trans).rot;
                }
                else if( xmlname == "controller" ) {
                }
                else if( xmlname == "jointvalues" ) {
                    _vjointvalues = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
                }

                if( xmlname !=_processingtag )
                    RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%xmlname%_processingtag));
                _processingtag = "";
            }
            else if( InterfaceXMLReader::endElement(xmlname) ) {
                if( _robotname.size() > 0 )
                    _probot->SetName(_robotname);
                if( _filename.size() > 0 ) {
                    SetXMLFilename(_filename);
                }

                // add prefix
                if( _prefix.size() > 0 ) {
                    vector<KinBody::LinkPtr>::iterator itlink = _probot->_veclinks.begin()+rootoffset;
                    while(itlink != _probot->_veclinks.end()) {
                        (*itlink)->name = _prefix + (*itlink)->name;
                        ++itlink;
                    }
                    vector<KinBody::JointPtr>::iterator itjoint = _probot->_vecjoints.begin()+rootjoffset;
                    while(itjoint != _probot->_vecjoints.end()) {
                        (*itjoint)->name = _prefix +(*itjoint)->name;
                        ++itjoint;
                    }
                    vector<RobotBase::AttachedSensorPtr>::iterator itsensor = _probot->GetAttachedSensors().begin()+rootsoffset;
                    while(itsensor != _probot->GetAttachedSensors().end()) {
                        (*itsensor)->_name = _prefix + (*itsensor)->_name;
                        ++itsensor;
                    }
                    vector<RobotBase::ManipulatorPtr>::iterator itmanip = _probot->GetManipulators().begin()+rootmoffset;
                    while(itmanip != _probot->GetManipulators().end()) {
                        (*itmanip)->_name = _prefix + (*itmanip)->_name;
                        ++itmanip;
                    }
                }
        
                // transform all "new" bodies with trans
                vector<KinBody::LinkPtr>::iterator itlink = _probot->_veclinks.begin()+rootoffset;
                while(itlink != _probot->_veclinks.end()) {
                    (*itlink)->SetTransform(_trans * (*itlink)->GetTransform());
                    ++itlink;
                }
        
                if( _vjointvalues.size() > 0 ) {
                    if( _vjointvalues.size() == _probot->GetJoints().size() ) {
                        _probot->SetJointValues(_vjointvalues);
                    }
                    else
                        RAVELOG_WARNA(str(boost::format("jointvalues for body %s wrong number (%d!=%d)\n")%_probot->GetName()%_vjointvalues.size()%_probot->GetJoints().size()));
                }

                // set a default controller
                if( !_probot->GetController() )
                    _probot->SetController(_probot->GetEnv()->CreateController("IdealController"),"");

                // forces robot to reupdate its internal objects
                _probot->SetTransform(_probot->GetTransform());
        
                if( !_probot->GetEnv()->GetPhysicsEngine()->InitKinBody(_probot) )
                    RAVELOG_WARNA("physics engine failed to init robot %s\n", _probot->GetName().c_str());

                return true;
            }
            return false;
        }

    protected:
        RobotBasePtr _probot;
        InterfaceBasePtr _pcontroller; ///< controller to set the robot at
        string _robotname;
        string _prefix;
        string _processingtag;

        vector<dReal> _vjointvalues;
        RobotBase::AttachedSensorPtr _psensor;

        Transform _trans;
        int rootoffset;                 ///< the initial number of links when Robot is created (so that global translations and rotations only affect the new links)
        int rootjoffset; ///< the initial number of joints when Robot is created
        int rootsoffset; ///< the initial number of attached sensors when Robot is created
        int rootmoffset; ///< the initial number of manipulators when Robot is created
    };

    template <InterfaceType type> class DummyInterfaceXMLReader : public InterfaceXMLReader
    {
    public:
    DummyInterfaceXMLReader(EnvironmentBasePtr penv, InterfaceBasePtr& pinterface, const string& xmltag, const std::list<std::pair<std::string,std::string> >& atts) : InterfaceXMLReader(penv,pinterface,type,xmltag,atts) {
        }
        virtual ~DummyInterfaceXMLReader() {}
    };

    class ProblemXMLReader : public InterfaceXMLReader
    {
    public:
        ProblemXMLReader(EnvironmentBasePtr penv, InterfaceBasePtr& pinterface, const std::list<std::pair<std::string,std::string> >& atts) : InterfaceXMLReader(penv,pinterface,PT_ProblemInstance,RaveGetInterfaceName(PT_ProblemInstance),atts) {
            FOREACHC(itatt,atts) {
                if( itatt->first == "args" )
                    _args = itatt->second;
            }

            if( !!_pinterface ) {
                ProblemInstancePtr problem = RaveInterfaceCast<ProblemInstance>(_pinterface);
                if( !!problem ) {
                    int ret = _penv->LoadProblem(problem,_args);
                    if( ret ) {
                        RAVELOG_WARN(str(boost::format("problem %s returned %d\n")%problem->GetXMLId()%ret));
                        problem.reset();
                        _pinterface.reset();
                    }
                }
            }
        }

        string _args;
    };

    class EnvironmentXMLReader : public StreamXMLReader
    {
    public:
    EnvironmentXMLReader(EnvironmentBasePtr penv, const std::list<std::pair<std::string,std::string> >& atts, bool bInEnvironment) : _penv(penv), _bInEnvironment(bInEnvironment)
        {
            if( !_penv ) {
                throw openrave_exception("need valid environment",ORE_InvalidArguments);
            }
            FOREACHC(itatt,atts) {
                if( itatt->first == "file" ) {
                    std::list<std::pair<std::string,std::string> > listnewatts;
                    FOREACHC(itatt2,atts) {
                        if( itatt2->first != "file" )
                            listnewatts.push_back(*itatt2);
                    }

                    boost::shared_ptr<pair<string,string> > filedata = RaveFindXMLFile(itatt->second);
                    if( !filedata ) {
                        continue;
                    }
                    _penv->Load(filedata->second);
                }
            }
            tCamera.trans = Vector(0, 1.5f, 0.8f);
            tCamera.rotfromaxisangle(Vector(1, 0, 0), (dReal)-0.5);
            vBkgndColor = Vector(1,1,1);
            bTransSpecified = false;
        }

        virtual ProcessElement startElement(const std::string& xmlname, const std::list<std::pair<std::string,std::string> >& atts)
        {
            switch( StreamXMLReader::startElement(xmlname,atts) ) {
                case PE_Pass: break;
                case PE_Support: return PE_Support;
                case PE_Ignore: return PE_Ignore;
            }
            if( _processingtag.size() > 0 ) {
                return PE_Ignore;
            }
            // check for any plugins
            FOREACHC(itname,RaveGetInterfaceNamesMap()) {
                if( xmlname == itname->second ) {
                    if( !!_pinterface )
                        throw openrave_exception("interface should not be initialized");
                    _pcurreader = CreateInterfaceReader(_penv,itname->first,_pinterface,"",atts);
                    if( !_pinterface ) {
                        RAVELOG_WARNA("failed to create interface %s in <environment>\n", itname->second.c_str());
                        _pcurreader.reset(new DummyXMLReader(xmlname,"environment"));
                    }
                    return PE_Support;
                }
            }

            if( xmlname == "environment" ) {
                _pcurreader.reset(new EnvironmentXMLReader(_penv,atts,true));
                return PE_Support;
            }

            if (xmlname == "bkgndcolor" || xmlname == "camrotaxis" || xmlname == "camrotationaxis" || xmlname == "camrotmat" || xmlname == "camtrans" || xmlname == "bkgndcolor" || xmlname == "plugin") {
                _processingtag = xmlname;
                return PE_Support;
            }
            return PE_Pass;
        }

        virtual bool endElement(const std::string& xmlname)
        {
            if( !!_pcurreader ) {
                if( _pcurreader->endElement(xmlname) ) {
                    if( !_bInEnvironment ) {
                        InterfaceXMLReaderPtr pinterfacereader = boost::dynamic_pointer_cast<InterfaceXMLReader>(_pcurreader);
                        if( !!pinterfacereader )
                            pinterfacereader->SetXMLFilename(_filename);
                    }

                    if( !!boost::dynamic_pointer_cast<RobotXMLReader>(_pcurreader) ) {
                        BOOST_ASSERT(_pinterface->GetInterfaceType()==PT_Robot);
                        _penv->AddRobot(RaveInterfaceCast<RobotBase>(_pinterface));
                        _pinterface.reset();
                    }
                    else if( !!boost::dynamic_pointer_cast<KinBodyXMLReader>(_pcurreader) ) {
                        BOOST_ASSERT(_pinterface->GetInterfaceType()==PT_KinBody);
                        _penv->AddKinBody(RaveInterfaceCast<KinBody>(_pinterface));
                        _pinterface.reset();
                    }
                    else if( !!boost::dynamic_pointer_cast< DummyInterfaceXMLReader<PT_PhysicsEngine> >(_pcurreader) ) {
                        BOOST_ASSERT(_pinterface->GetInterfaceType()==PT_PhysicsEngine);
                        _penv->SetPhysicsEngine(RaveInterfaceCast<PhysicsEngineBase>(_pinterface));
                        _pinterface.reset();
                    }
                    else if( !!boost::dynamic_pointer_cast< DummyInterfaceXMLReader<PT_CollisionChecker> >(_pcurreader) ) {
                        BOOST_ASSERT(_pinterface->GetInterfaceType()==PT_CollisionChecker);
                        _penv->SetCollisionChecker(RaveInterfaceCast<CollisionCheckerBase>(_pinterface));
                        _pinterface.reset();
                    }
                    else if( !!_pinterface ) {
                        RAVELOG_DEBUGA("owning interface %s, type: %s\n",_pinterface->GetXMLId().c_str(),RaveGetInterfaceName(_pinterface->GetInterfaceType()).c_str());
                        _penv->OwnInterface(_pinterface);
                        _pinterface.reset();
                    }
                    _pcurreader.reset();
                }
                return false;
            }
            if( xmlname == "environment" ) {
                // only move the camera if trans is specified
                if( !!_penv->GetViewer() ) {
                    if( bTransSpecified ) {
                        _penv->GetViewer()->SetCamera(tCamera);
                    }
                    _penv->GetViewer()->SetBkgndColor(vBkgndColor);
                }
                return true;
            }
            
            if( xmlname == "bkgndcolor" ) {
                _ss >> vBkgndColor.x >> vBkgndColor.y >> vBkgndColor.z;
            }
            else if( xmlname == "camrotaxis" || xmlname == "camrotationaxis" ) {
                Vector vaxis; dReal fangle=0;
                _ss >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
                tCamera.rotfromaxisangle(vaxis.normalize3(), fangle * PI / 180.0f);
                bTransSpecified = true;
            }
            else if( xmlname == "camrotmat" ) {
                TransformMatrix tnew;
                _ss >> tnew.m[0] >> tnew.m[1] >> tnew.m[2] >> tnew.m[4] >> tnew.m[5] >> tnew.m[6] >> tnew.m[8] >> tnew.m[9] >> tnew.m[10];
                tCamera.rot = Transform(tnew).rot;
                bTransSpecified = true;
            }
            else if( xmlname == "camtrans" ) {
                _ss >> tCamera.trans.x >> tCamera.trans.y >> tCamera.trans.z;
                bTransSpecified = true;
            }
            else if( xmlname == "plugin" ) {
                string pluginname;
                _ss >> pluginname;
                _penv->LoadPlugin(pluginname);
            }

            if( xmlname !=_processingtag )
                RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%xmlname%_processingtag));
            _processingtag = "";
            return false;
        }

    protected:
        EnvironmentBasePtr _penv;
        InterfaceBasePtr _pinterface; // current processed interface
        Vector vBkgndColor;
        Transform tCamera; ///< default camera transformationn
        string _processingtag;
        bool bTransSpecified;
        bool _bInEnvironment;
    };

    static InterfaceXMLReaderPtr CreateInterfaceReader(EnvironmentBasePtr penv, InterfaceType type, InterfaceBasePtr& pinterface, const std::string& xmltag, const std::list<std::pair<std::string,std::string> >& atts)
    {
        switch(type) {
        case PT_Planner: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_Planner>(penv,pinterface,xmltag,atts));
        case PT_Robot: {
            RobotBasePtr probot = RaveInterfaceCast<RobotBase>(pinterface);
            int rootoffset = 0, rootjoffset = 0, rootsoffset = 0, rootmoffset = 0;
            if( !!probot ) {
                rootoffset = (int)probot->GetLinks().size();
                rootjoffset = (int)probot->GetJoints().size();
                rootsoffset = (int)probot->GetAttachedSensors().size();
                rootmoffset = (int)probot->GetManipulators().size();
            }
            return InterfaceXMLReaderPtr(new RobotXMLReader(penv,pinterface,atts,rootoffset,rootjoffset,rootsoffset,rootmoffset));
        }
        case PT_SensorSystem: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_SensorSystem>(penv,pinterface,xmltag,atts));
        case PT_Controller: return InterfaceXMLReaderPtr(new ControllerXMLReader(penv,pinterface,atts));
        case PT_ProblemInstance: return InterfaceXMLReaderPtr(new ProblemXMLReader(penv,pinterface,atts));
        case PT_InverseKinematicsSolver: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_InverseKinematicsSolver>(penv,pinterface,xmltag,atts));
        case PT_KinBody: {
            KinBodyPtr pbody = RaveInterfaceCast<KinBody>(pinterface);
            int rootoffset = 0, rootjoffset = 0;
            if( !!pbody ) {
                vector<Transform> vTransforms;
                pbody->GetBodyTransformations(vTransforms);
                rootoffset = vTransforms.size();
                rootjoffset = (int)pbody->GetJoints().size();
            }
            return InterfaceXMLReaderPtr(new KinBodyXMLReader(penv,pinterface,type,atts,rootoffset,rootjoffset));
        }
        case PT_PhysicsEngine: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_PhysicsEngine>(penv,pinterface,xmltag,atts));
        case PT_Sensor: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_Sensor>(penv,pinterface,xmltag,atts));
        case PT_CollisionChecker: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_CollisionChecker>(penv,pinterface,xmltag,atts));
        case PT_Trajectory: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_Trajectory>(penv,pinterface,xmltag,atts));
        case PT_Viewer: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_Viewer>(penv,pinterface,xmltag,atts));
        }

        throw openrave_exception(str(boost::format("could not create interface of type %d")%type),ORE_InvalidArguments);
    }

    class GlobalInterfaceXMLReader : public StreamXMLReader
    {
    public:
    GlobalInterfaceXMLReader(EnvironmentBasePtr penv) : _penv(penv) {}
        virtual ProcessElement startElement(const std::string& xmlname, const std::list<std::pair<std::string,std::string> >& atts)
        {
            switch( StreamXMLReader::startElement(xmlname,atts) ) {
                case PE_Pass: break;
                case PE_Support: return PE_Support;
                case PE_Ignore: return PE_Ignore;
            }
                        
            // check for any plugins
            FOREACHC(itname,RaveGetInterfaceNamesMap()) {
                if( xmlname == itname->second ) {
                    if( !!_pinterface )
                        throw openrave_exception("interface should not be initialized");
                    _pcurreader = CreateInterfaceReader(_penv,itname->first,_pinterface,"",atts);
                    if( !_pinterface )
                        throw openrave_exception(str(boost::format("failed to create interface %s")%itname->second));
                    return PE_Support;
                }
            }

            throw openrave_exception(str(boost::format("invalid interface tag %s")%xmlname));
        }

        virtual bool endElement(const std::string& xmlname)
        {
            if( !!_pcurreader ) {
                if( _pcurreader->endElement(xmlname) ) {
                    _pcurreader.reset();
                    // end if current reader is an interface
                    return !!boost::dynamic_pointer_cast<InterfaceXMLReader>(_pcurreader);
                }
            }
            return false;
        }

        InterfaceBasePtr GetInterface() { return _pinterface; }
    protected:
        EnvironmentBasePtr _penv;
        InterfaceBasePtr _pinterface; // current processed interface
    };
};

#endif
