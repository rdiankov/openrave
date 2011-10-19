// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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

#include <libxml/xmlstring.h>
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
#include <boost/filesystem.hpp>
#endif

#include <boost/utility.hpp>
#include <boost/thread/once.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

BOOST_STATIC_ASSERT(sizeof(xmlChar) == 1);

#ifdef OPENRAVE_ASSIMP
#include <assimp.hpp>
#include <aiScene.h>
#include <aiPostProcess.h>
#endif

#ifdef OPENRAVE_IVCON
#include <ivcon.h>
#endif

namespace OpenRAVEXMLParser
{

static boost::once_flag __onceCreateXMLMutex = BOOST_ONCE_INIT;
/// lock for parsing XML, don't destroy it in order to ensure it remains valid for as long as possible
static EnvironmentMutex* __mutexXML;
void __CreateXMLMutex()
{
    __mutexXML = new EnvironmentMutex();
}

EnvironmentMutex* GetXMLMutex()
{
    boost::call_once(__CreateXMLMutex,__onceCreateXMLMutex);
    return __mutexXML;
}

/// the directory of the file currently parsing
std::string& GetParseDirectory() {
    static string s; return s;
}
/// full filename currently parsing
std::string& GetFullFilename() {
    static string s; return s;
}

std::vector<string>& RaveGetDataDirs()
{
    static vector<string> v;
    return v;
}

int& GetXMLErrorCount()
{
    static int errorcount=0;
    return errorcount;
}

void SetDataDirs(const vector<string>& vdatadirs) {
    EnvironmentMutex::scoped_lock lock(*GetXMLMutex());
    RaveGetDataDirs() = vdatadirs;
}

#ifdef OPENRAVE_ASSIMP
class aiSceneManaged
{
public:
    aiSceneManaged(const std::string& filename, unsigned int flags = aiProcess_JoinIdenticalVertices|aiProcess_Triangulate|aiProcess_FindDegenerates|aiProcess_PreTransformVertices|aiProcess_SortByPType) {
        _importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT|aiPrimitiveType_LINE);
        _scene = _importer.ReadFile(filename.c_str(),flags);
        if( _scene == NULL ) {
            RAVELOG_VERBOSE("assimp error: %s\n",_importer.GetErrorString());
        }
    }
    virtual ~aiSceneManaged() {
        _importer.FreeScene();
    }
    Assimp::Importer _importer;
    const struct aiScene* _scene;
};

static bool _AssimpCreateTriMesh(const aiScene* scene, aiNode* node, const Vector& scale, KinBody::Link::TRIMESH& trimesh, RaveVector<float>& diffuseColor, RaveVector<float>& ambientColor, float& ftransparency)
{
    if( !node ) {
        return false;
    }
    aiMatrix4x4 transform = node->mTransformation;
    aiNode *pnode = node->mParent;
    while (pnode) {
        // Don't convert to y-up orientation, which is what the root node is in, Assimp does
        if (pnode->mParent != NULL) {
            transform = pnode->mTransformation * transform;
        }
        pnode = pnode->mParent;
    }

    std::vector<Vector>& vertices = trimesh.vertices;
    std::vector<int>& indices = trimesh.indices;
    {
        size_t vertexOffset = vertices.size();
        size_t nTotalVertices=0;
        for (size_t i = 0; i < node->mNumMeshes; i++) {
            aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
            nTotalVertices += input_mesh->mNumVertices;
        }

        vertices.reserve(vertices.size()+nTotalVertices);
        for (size_t i = 0; i < node->mNumMeshes; i++) {
            aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
            for (size_t j = 0; j < input_mesh->mNumVertices; j++) {
                aiVector3D p = input_mesh->mVertices[j];
                p *= transform;
                vertices.push_back(Vector(p.x*scale.x,p.y*scale.y,p.z*scale.z));
            }
        }
        for (size_t i = 0; i < node->mNumMeshes; i++) {
            aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
            size_t indexCount = 0;
            for (size_t j = 0; j < input_mesh->mNumFaces; j++) {
                aiFace& face = input_mesh->mFaces[j];
                indexCount += 3*(face.mNumIndices-2);
            }
            indices.reserve(indices.size()+indexCount);
            for (size_t j = 0; j < input_mesh->mNumFaces; j++) {
                aiFace& face = input_mesh->mFaces[j];
                if( face.mNumIndices == 3 ) {
                    indices.push_back(vertexOffset+face.mIndices[0]);
                    indices.push_back(vertexOffset+face.mIndices[1]);
                    indices.push_back(vertexOffset+face.mIndices[2]);
                }
                else {
                    for (size_t k = 2; k < face.mNumIndices; ++k) {
                        indices.push_back(face.mIndices[0]+vertexOffset);
                        indices.push_back(face.mIndices[k-1]+vertexOffset);
                        indices.push_back(face.mIndices[k]+vertexOffset);
                    }
                }
            }
            vertexOffset += input_mesh->mNumVertices;
        }
    }
    for (size_t i=0; i < node->mNumChildren; ++i) {
        _AssimpCreateTriMesh(scene, node->mChildren[i], scale, trimesh, diffuseColor, ambientColor, ftransparency);
    }
    return true;
}

#endif

bool CreateTriMeshData(EnvironmentBasePtr penv, const std::string& filename, const Vector& vscale, KinBody::Link::TRIMESH& trimesh, RaveVector<float>& diffuseColor, RaveVector<float>& ambientColor, float& ftransparency)
{
    string extension;
    if( filename.find_last_of('.') != string::npos ) {
        extension = filename.substr(filename.find_last_of('.')+1);
        std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
    }

#ifdef OPENRAVE_ASSIMP
    // assimp doesn't support vrml/iv, so don't waste time
    if((extension != "iv")&&(extension != "wrl")&&(extension != "vrml")) {
        aiSceneManaged scene(filename);
        if( !!scene._scene && !!scene._scene->mRootNode && !!scene._scene->HasMeshes() ) {
            if( _AssimpCreateTriMesh(scene._scene,scene._scene->mRootNode, vscale, trimesh, diffuseColor, ambientColor, ftransparency) ) {
                return true;
            }
        }
        if( extension == "stl" || extension == "x") {
            return false;
        }
    }
#endif

    ModuleBasePtr ivmodelloader = RaveCreateModule(penv,"ivmodelloader");
    if( !!ivmodelloader ) {
        stringstream sout, sin;
        sin << "LoadModel " << filename;
        sout << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        if( ivmodelloader->SendCommand(sout,sin) ) {
            sout >> trimesh >> diffuseColor >> ambientColor >> ftransparency;
            if( !!sout ) {
                FOREACH(it,trimesh.vertices) {
                    it->x *= vscale.x;
                    it->y *= vscale.y;
                    it->z *= vscale.z;
                }
                return true;
            }
        }
    }

#ifdef OPENRAVE_IVCON
    RAVELOG_DEBUG("using ivcon for geometry reading\n");
    vector<float> vertices;
    if( ivcon::ReadFile(filename.c_str(), vertices, trimesh.indices) ) {
        trimesh.vertices.resize(vertices.size()/3);
        for(size_t i = 0; i < vertices.size(); i += 3) {
            trimesh.vertices[i/3] = Vector(vscale.x*vertices[i],vscale.y*vertices[i+1],vscale.z*vertices[i+2]);
        }
        return true;
    }
#endif
    return false;
}


struct XMLREADERDATA
{
    XMLREADERDATA(BaseXMLReaderPtr preader, xmlParserCtxtPtr ctxt) : _preader(preader), _ctxt(ctxt) {
    }
    BaseXMLReaderPtr _preader, _pdummy;
    xmlParserCtxtPtr _ctxt;
};

static void DefaultStartElementSAXFunc(void *ctx, const xmlChar *name, const xmlChar **atts)
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
    if( !!pdata->_pdummy ) {
        pdata->_pdummy->characters(string((const char*)ch, len));
    }
    else {
        pdata->_preader->characters(string((const char*)ch, len));
    }
}

static void RaveXMLErrorFunc(void *ctx, const char *msg, ...)
{
    va_list args;
    va_start(args, msg);
    RAVELOG_ERROR("XML Parse error: ");
    vprintf(msg,args);
    va_end(args);
    GetXMLErrorCount()++;
}

static xmlSAXHandler* GetSAXHandler()
{
    static xmlSAXHandler s_DefaultSAXHandler = { 0};
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
    if ((ctxt->sax) &&  (ctxt->sax->initialized == XML_SAX2_MAGIC) && ((ctxt->sax->startElementNs != NULL) || (ctxt->sax->endElementNs != NULL))) {
        ctxt->sax2 = 1;
    }
#else
    ctxt->sax2 = 1;
#endif // LIBXML_SAX1_ENABLED

    ctxt->str_xml = xmlDictLookup(ctxt->dict, BAD_CAST "xml", 3);
    ctxt->str_xmlns = xmlDictLookup(ctxt->dict, BAD_CAST "xmlns", 5);
    ctxt->str_xml_ns = xmlDictLookup(ctxt->dict, XML_XML_NAMESPACE, 36);
    if ((ctxt->str_xml==NULL) || (ctxt->str_xmlns==NULL) || (ctxt->str_xml_ns == NULL)) {
        return false;
    }
    return true;
}

static int raveXmlSAXUserParseFile(xmlSAXHandlerPtr sax, BaseXMLReaderPtr preader, const std::string& filename)
{
    int ret = 0;
    xmlParserCtxtPtr ctxt;
    ctxt = xmlCreateFileParserCtxt(filename.c_str());
    if (ctxt == NULL) {
        return -1;
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
    return ret;
}

static int raveXmlSAXUserParseMemory(xmlSAXHandlerPtr sax, BaseXMLReaderPtr preader, const char *buffer, int size)
{
    int ret = 0;
    xmlParserCtxtPtr ctxt;

    ctxt = xmlCreateMemoryParserCtxt(buffer, size);
    if (ctxt == NULL) {
        return -1;
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
    return ret;
}

boost::shared_ptr<std::pair<std::string,std::string> > FindFile(const std::string& filename)
{
    if( filename.size() == 0 ) {
        return boost::shared_ptr<pair<string,string> >();
    }
    EnvironmentMutex::scoped_lock lock(*GetXMLMutex());

    // init the dir (for some reason msvc confuses the linking of the string class with soqt, so do it the old fashioned way)
    string appended;
    size_t sepindex = filename.find_last_of('/');
    if( sepindex == string::npos ) {
        sepindex = filename.find_last_of('\\');
    }
    string parsedirectory = GetParseDirectory(), fullfilename = GetFullFilename();

    // check for absolute paths
#ifdef _WIN32
    if((filename.size() > 2)&&(filename[1] == ':')) {
#else
    if((filename[0] == '/')||(filename[0] == '~')) {
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
        FOREACHC(itdir, RaveGetDataDirs()) {
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
        if( bFileFound ) {
            break;
        }

    } while(0);

    if( !bFileFound ) {
        GetXMLErrorCount()++;
        RAVELOG_WARN(str(boost::format("could not find file %s\n")%filename));
        return boost::shared_ptr<pair<string,string> >();
    }

#ifdef HAVE_BOOST_FILESYSTEM
    fullfilename = boost::filesystem::system_complete(boost::filesystem::path(fullfilename)).string();
#endif
    return boost::shared_ptr<pair<string,string> >(new pair<string,string>(parsedirectory,fullfilename));
}

bool ParseXMLFile(BaseXMLReaderPtr preader, const string& filename)
{
    boost::shared_ptr<pair<string,string> > filedata = FindFile(filename);
    if( !filedata ) {
        return false;
    }
    EnvironmentMutex::scoped_lock lock(*GetXMLMutex());

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
    catch(const openrave_exception &ex) {
        RAVELOG_ERROR(str(boost::format("xmlSAXUserParseFile: error parsing %s: %s\n")%GetFullFilename()%ex.what()));
        ret = -1;
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

bool ParseXMLData(BaseXMLReaderPtr preader, const std::string& pdata)
{
    if( pdata.size() == 0 ) {
        return false;
    }
    EnvironmentMutex::scoped_lock lock(*GetXMLMutex());
    return raveXmlSAXUserParseMemory(GetSAXHandler(), preader, pdata.c_str(), pdata.size())==0;
}

/// mass of objects
struct MASS
{
    MASS() : fTotalMass(0) {
        for(int i = 0; i < 12; ++i) t.m[i] = 0;
    }
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
    MASS operator+(const MASS &r) const
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
    MASS& operator+=(const MASS &r)
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
        t = trot * t * trot.inverse();     // rotate mass about rotation

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
    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList& atts)
    {
        _ss.str("");         // have to clear the string
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
        if( !!_pcurreader ) {
            _pcurreader->characters(ch);
        }
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
#ifdef OPENRAVE_ASSIMP
    static bool _AssimpCreateGeometries(const aiScene* scene, aiNode* node, const Vector& scale, std::list<KinBody::Link::GEOMPROPERTIES>& listGeometries)
    {
        if( !node ) {
            return false;
        }
        aiMatrix4x4 transform = node->mTransformation;
        aiNode *pnode = node->mParent;
        while (pnode) {
            // Don't convert to y-up orientation, which is what the root node is in, Assimp does
            if (pnode->mParent != NULL) {
                transform = pnode->mTransformation * transform;
            }
            pnode = pnode->mParent;
        }

        for (size_t i = 0; i < node->mNumMeshes; i++) {
            listGeometries.push_back(KinBody::Link::GEOMPROPERTIES(KinBody::LinkPtr()));
            KinBody::Link::GEOMPROPERTIES& g = listGeometries.back();
            g._type = KinBody::Link::GEOMPROPERTIES::GeomTrimesh;
            aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
            g.collisionmesh.vertices.resize(input_mesh->mNumVertices);
            for (size_t j = 0; j < input_mesh->mNumVertices; j++) {
                aiVector3D p = input_mesh->mVertices[j];
                p *= transform;
                g.collisionmesh.vertices[j] = Vector(p.x*scale.x,p.y*scale.y,p.z*scale.z);
            }
            size_t indexCount = 0;
            for (size_t j = 0; j < input_mesh->mNumFaces; j++) {
                aiFace& face = input_mesh->mFaces[j];
                indexCount += 3*(face.mNumIndices-2);
            }
            g.collisionmesh.indices.reserve(indexCount);
            for (size_t j = 0; j < input_mesh->mNumFaces; j++) {
                aiFace& face = input_mesh->mFaces[j];
                if( face.mNumIndices == 3 ) {
                    g.collisionmesh.indices.push_back(face.mIndices[0]);
                    g.collisionmesh.indices.push_back(face.mIndices[1]);
                    g.collisionmesh.indices.push_back(face.mIndices[2]);
                }
                else {
                    for (size_t k = 2; k < face.mNumIndices; ++k) {
                        g.collisionmesh.indices.push_back(face.mIndices[0]);
                        g.collisionmesh.indices.push_back(face.mIndices[k-1]);
                        g.collisionmesh.indices.push_back(face.mIndices[k]);
                    }
                }
            }

            if( !!scene->mMaterials && input_mesh->mMaterialIndex>=0 && input_mesh->mMaterialIndex<scene->mNumMaterials) {
                aiMaterial* mtrl = scene->mMaterials[input_mesh->mMaterialIndex];
                aiColor4D color;
                aiGetMaterialColor(mtrl,AI_MATKEY_COLOR_DIFFUSE,&color);
                g.diffuseColor = Vector(color.r,color.g,color.b,color.a);
                aiGetMaterialColor(mtrl,AI_MATKEY_COLOR_AMBIENT,&color);
                g.ambientColor = Vector(color.r,color.g,color.b,color.a);
            }
        }

        for (size_t i=0; i < node->mNumChildren; ++i) {
            _AssimpCreateGeometries(scene, node->mChildren[i], scale, listGeometries);
        }
        return true;
    }
#endif

    static bool CreateGeometries(EnvironmentBasePtr penv, const std::string& filename, const Vector& vscale, std::list<KinBody::Link::GEOMPROPERTIES>& listGeometries)
    {
        string extension;
        if( filename.find_last_of('.') != string::npos ) {
            extension = filename.substr(filename.find_last_of('.')+1);
            std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
        }

#ifdef OPENRAVE_ASSIMP
        // assimp doesn't support vrml/iv, so don't waste time
        if( extension != "iv" && extension != "wrl" && extension != "vrml" ) {
            aiSceneManaged scene(filename);
            if( !!scene._scene && !!scene._scene->mRootNode && !!scene._scene->HasMeshes() ) {
                if( _AssimpCreateGeometries(scene._scene,scene._scene->mRootNode, vscale, listGeometries) ) {
                    return true;
                }
            }
            if( extension == "stl" || extension == "x") {
                return false;
            }
        }
#endif

        // for other importers, just convert into one big trimesh
        listGeometries.push_back(KinBody::Link::GEOMPROPERTIES(KinBody::LinkPtr()));
        KinBody::Link::GEOMPROPERTIES& g = listGeometries.back();
        g._type = KinBody::Link::GEOMPROPERTIES::GeomTrimesh;
        g.diffuseColor=Vector(1,0.5f,0.5f,1);
        g.ambientColor=Vector(0.1,0.0f,0.0f,0);
        if( !CreateTriMeshData(penv,filename,vscale,g.collisionmesh,g.diffuseColor,g.ambientColor,g.ftransparency) ) {
            return false;
        }
        return true;
    }

    enum MassType
    {
        MT_None = 0,
        MT_MimicGeom,
        MT_Box,
        MT_BoxMass,         // use total mass instead of density
        MT_Sphere,
        MT_SphereMass,
        MT_Custom,         // manually specify center of mass and inertia matrix
    };

    LinkXMLReader(KinBody::LinkPtr& plink, KinBodyPtr pparent, const AttributesList &atts) : _plink(plink) {
        _pparent = pparent;
        _masstype = MT_None;
        _fMassDensity = 1;
        _vMassExtents = Vector(1,1,1);
        _fTotalMass = 1;
        _massCustom = MASS::GetSphericalMass(1,Vector(0,0,0),1);
        _bSkipGeometry = false;
        _vScaleGeometry = Vector(1,1,1);
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
            else if( itatt->first == "file" ) {
                linkfilename = itatt->second;
            }
            else if( itatt->first == "skipgeometry" ) {
                _bSkipGeometry = stricmp(itatt->second.c_str(), "true") == 0 || itatt->second=="1";
            }
            else if( itatt->first == "scalegeometry" ) {
                stringstream ss(itatt->second);
                Vector v(1,1,1);
                ss >> v.x >> v.y >> v.z;
                if( !ss ) {
                    v.z = v.y = v.x;
                }
                _vScaleGeometry *= v;
            }
        }

        // if not appending to a body and plink pointer valid, append to it instead
        if( !_plink && !!plink &&( plink->GetParent() == pparent) ) {
            _plink = plink;
        }
        if( linkfilename.size() > 0 ) {
            ParseXMLFile(BaseXMLReaderPtr(new LinkXMLReader(_plink, _pparent, AttributesList())), linkfilename);
        }

        if( !_plink ) {
            _plink.reset(new KinBody::Link(pparent));
        }
        if( linkname.size() > 0 ) {
            _plink->_name = linkname;
        }
        if( bStaticSet ) {
            _plink->_bStatic = bStatic;
        }
        _itgeomprop = _plink->_listGeomProperties.end();
    }
    virtual ~LinkXMLReader() {
    }

    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList& atts)
    {
        switch( StreamXMLReader::startElement(xmlname,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }

        if( _processingtag.size() > 0 ) {
            if( _processingtag == "geom" ) {
                if( xmlname == "render" ) {
                    // check the attributes first
                    FOREACHC(itatt,atts) {
                        if( itatt->first == "file" ) {
                            _renderfilename.first = !_fnGetModelsDir ? itatt->second : _fnGetModelsDir(itatt->second);
                        }
                        else if( itatt->first == "scale" ) {
                            Vector vscale(1,1,1);
                            stringstream sslocal(itatt->second);
                            sslocal >> vscale.x; vscale.y = vscale.z = vscale.x;
                            sslocal >> vscale.y >> vscale.z;
                            _renderfilename.second = vscale;
                        }
                    }
                }
                else if( xmlname == "collision" ) {
                    // check the attributes first
                    FOREACHC(itatt,atts) {
                        if( itatt->first == "file" ) {
                            _collisionfilename.first = !_fnGetModelsDir ? itatt->second : _fnGetModelsDir(itatt->second);
                        }
                        else if( itatt->first == "scale" ) {
                            Vector vscale(1,1,1);
                            stringstream sslocal(itatt->second);
                            sslocal >> vscale.x; vscale.y = vscale.z = vscale.x;
                            sslocal >> vscale.y >> vscale.z;
                            _collisionfilename.second = vscale;
                        }
                    }
                }

                static boost::array<string,11> tags = { { "translation", "rotationmat", "rotationaxis", "quat", "diffusecolor", "ambientcolor", "transparency", "render", "extents", "radius", "height"}};
                if( find(tags.begin(),tags.end(),xmlname) != tags.end() ) {
                    return PE_Support;
                }
                switch(_itgeomprop->GetType()) {
                case KinBody::Link::GEOMPROPERTIES::GeomTrimesh:
                    if(( xmlname=="collision") ||( xmlname=="data") ||( xmlname=="vertices") ) {
                        return PE_Support;
                    }
                    break;
                default:
                    break;
                }
                return PE_Ignore;
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
            AttributesList newatts = atts;
            newatts.push_back(make_pair("skipgeometry",_bSkipGeometry ? "1" : "0"));
            newatts.push_back(make_pair("scalegeometry",str(boost::format("%f %f %f")%_vScaleGeometry.x%_vScaleGeometry.y%_vScaleGeometry.z)));
            _pcurreader.reset(new LinkXMLReader(_plink, _pparent, newatts));
            return PE_Support;
        }

        _processingtag = xmlname;
        if( xmlname == "geom" ) {
            if( _bSkipGeometry ) {
                _processingtag = "";
                return PE_Ignore;
            }

            string type;
            bool bDraw = true, bModifiable = true;
            FOREACHC(itatt,atts) {
                if( itatt->first == "type") {
                    type = itatt->second;
                }
                else if( itatt->first == "render" ) {
                    // set draw to false only if atts[i]==false
                    bDraw = stricmp(itatt->second.c_str(), "false")!=0 && itatt->second!="0";
                }
                else if( itatt->first == "modifiable" ) {
                    bModifiable = !(stricmp(itatt->second.c_str(), "false") == 0 || itatt->second=="0");
                }
            }

            if( type.size() == 0 ) {
                RAVELOG_INFOA("no geometry type, defaulting to box\n");
                type = "box";
            }

            _itgeomprop = _plink->_listGeomProperties.insert(_plink->_listGeomProperties.end(),KinBody::Link::GEOMPROPERTIES(_plink));
            if( stricmp(type.c_str(), "box") == 0 ) {
                _itgeomprop->_type = KinBody::Link::GEOMPROPERTIES::GeomBox;
            }
            else if( stricmp(type.c_str(), "sphere") == 0 ) {
                _itgeomprop->_type = KinBody::Link::GEOMPROPERTIES::GeomSphere;
            }
            else if( stricmp(type.c_str(), "cylinder") == 0 ) {
                _itgeomprop->_type = KinBody::Link::GEOMPROPERTIES::GeomCylinder;
            }
            else if( stricmp(type.c_str(), "trimesh") == 0 ) {
                _itgeomprop->_type = KinBody::Link::GEOMPROPERTIES::GeomTrimesh;
            }
            else {
                RAVELOG_WARN(str(boost::format("type %s not supported\n")%type));
            }
            _renderfilename.first.resize(0);
            _collisionfilename.first.resize(0);
            _itgeomprop->_bDraw = bDraw;
            _itgeomprop->_bModifiable = bModifiable;
            return PE_Support;
        }
        else if((xmlname == "translation")||(xmlname == "rotationmat")||(xmlname == "rotationaxis")||(xmlname == "quat")||(xmlname == "offsetfrom")) {
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
                _ss >>v.x >> v.y >> v.z;
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
                Transform tnew; tnew.rot = quatFromAxisAngle(vaxis, fangle * PI / 180.0f);
                _itgeomprop->_t.rot = (tnew*_itgeomprop->_t).rot;
            }
            else if( xmlname == "quat" ) {
                Transform tnew;
                _ss >> tnew.rot.x >> tnew.rot.y >> tnew.rot.z >> tnew.rot.w;
                tnew.rot.normalize4();
                _itgeomprop->_t.rot = (tnew*_itgeomprop->_t).rot;
            }
            else if( xmlname == "render" ) {
                if( _renderfilename.first.size() == 0 ) {
                    string orgfilename;
                    Vector vscale(1,1,1);
                    _ss >> orgfilename;
                    _ss >> vscale.x; vscale.y = vscale.z = vscale.x;
                    _ss >> vscale.y >> vscale.z;
                    _renderfilename.first = !_fnGetModelsDir ? orgfilename : _fnGetModelsDir(orgfilename);
                    _renderfilename.second = vscale;
                }
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
                TransformMatrix tminv(_itgeomprop->_t.inverse());
                TransformMatrix tm(_itgeomprop->_t);
                tm.m[0] *= _vScaleGeometry.x; tm.m[1] *= _vScaleGeometry.y; tm.m[2] *= _vScaleGeometry.z;
                tm.m[4] *= _vScaleGeometry.x; tm.m[5] *= _vScaleGeometry.y; tm.m[6] *= _vScaleGeometry.z;
                tm.m[8] *= _vScaleGeometry.x; tm.m[9] *= _vScaleGeometry.y; tm.m[10] *= _vScaleGeometry.z;
                tm.trans *= _vScaleGeometry;
                TransformMatrix tmres = tminv * tm;
                // have to scale in link space, so get scale in geomspace
                Vector geomspacescale(RaveSqrt(tm.m[0]*tm.m[0]+tm.m[1]*tm.m[1]+tm.m[2]*tm.m[2]),RaveSqrt(tm.m[4]*tm.m[4]+tm.m[5]*tm.m[5]+tm.m[6]*tm.m[6]),RaveSqrt(tm.m[8]*tm.m[8]+tm.m[9]*tm.m[9]+tm.m[10]*tm.m[10]));

                std::list<KinBody::Link::GEOMPROPERTIES> listGeometries;
                if( _itgeomprop->GetType() == KinBody::Link::GEOMPROPERTIES::GeomTrimesh ) {
                    bool bSuccess = false;
                    if( _collisionfilename.first.size() > 0 ) {
                        _itgeomprop->vRenderScale = _renderfilename.second*geomspacescale;
                        _itgeomprop->_renderfilename = _renderfilename.first;
                        if( !CreateGeometries(_pparent->GetEnv(),_collisionfilename.first, _collisionfilename.second, listGeometries) ) {
                            RAVELOG_WARN(str(boost::format("failed to find %s\n")%_collisionfilename.first));
                        }
                        else {
                            bSuccess = true;
                        }
                    }
                    if( _renderfilename.first.size() > 0 ) {
                        if( !bSuccess ) {
                            if( !CreateGeometries(_pparent->GetEnv(), _renderfilename.first, _renderfilename.second, listGeometries) ) {
                                RAVELOG_WARN(str(boost::format("failed to find %s\n")%_renderfilename.first));
                            }
                            else {
                                bSuccess = true;
                            }
                        }
                    }

                    if( listGeometries.size() > 0 ) {
                        // append all the geometries to the link. make sure the render filename is specified in only one geometry.
                        string extension;
                        if( _renderfilename.first.find_last_of('.') != string::npos ) {
                            extension = _renderfilename.first.substr(_renderfilename.first.find_last_of('.')+1);
                        }
                        FOREACH(itnewgeom,listGeometries) {
                            itnewgeom->_parent = _plink;
                            itnewgeom->_bDraw = _itgeomprop->_bDraw;
                            itnewgeom->_bModifiable = _itgeomprop->_bModifiable;
                            itnewgeom->_t = _itgeomprop->_t;
                            itnewgeom->_parent = _itgeomprop->_parent;
                            itnewgeom->ftransparency = _itgeomprop->ftransparency;
                            itnewgeom->_renderfilename = string("__norenderif__:")+extension;
                            FOREACH(it,itnewgeom->collisionmesh.vertices) {
                                *it = tmres * *it;
                            }
                            _plink->collision.Append(itnewgeom->GetCollisionMesh(), itnewgeom->_t);
                            itnewgeom->_t.trans *= _vScaleGeometry;
                        }
                        listGeometries.front().vRenderScale = _renderfilename.second*geomspacescale;
                        listGeometries.front()._renderfilename = _renderfilename.first;
                        listGeometries.front()._bDraw = _itgeomprop->_bDraw;
                        _plink->_listGeomProperties.erase(_itgeomprop);
                        _plink->_listGeomProperties.splice(_plink->_listGeomProperties.end(),listGeometries);
                    }
                    else {
                        _itgeomprop->vRenderScale = _renderfilename.second*geomspacescale;
                        _itgeomprop->_renderfilename = _renderfilename.first;
                        FOREACH(it,_itgeomprop->collisionmesh.vertices) {
                            *it = tmres * *it;
                        }
                        _plink->collision.Append(_itgeomprop->GetCollisionMesh(), _itgeomprop->_t);
                        _itgeomprop->_t.trans *= _vScaleGeometry;
                    }
                }
                else {
                    _itgeomprop->vRenderScale = _renderfilename.second*_vScaleGeometry;
                    _itgeomprop->_renderfilename = _renderfilename.first;

                    if( _itgeomprop->GetType() == KinBody::Link::GEOMPROPERTIES::GeomCylinder ) {         // axis has to point on y
                        // rotate on x axis by pi/2
                        Transform trot;
                        trot.rot = quatFromAxisAngle(Vector(1, 0, 0), PI/2);
                        _itgeomprop->_t.rot = (_itgeomprop->_t*trot).rot;
                    }

                    // call before attaching the geom
                    _itgeomprop->InitCollisionMesh();
                    FOREACH(it,_itgeomprop->collisionmesh.vertices) {
                        *it = tmres * *it;
                    }
                    _plink->collision.Append(_itgeomprop->GetCollisionMesh(), _itgeomprop->_t);
                    _itgeomprop->_t.trans *= _vScaleGeometry;
                    _itgeomprop->vGeomData *= geomspacescale;
                }

                _itgeomprop = _plink->_listGeomProperties.end();
                _processingtag = "";
            }
            else {
                // could be type specific features
                switch(_itgeomprop->GetType()) {
                case KinBody::Link::GEOMPROPERTIES::GeomSphere:
                    if( xmlname == "radius" ) {
                        _ss >> _itgeomprop->vGeomData.x;
                    }
                    break;
                case KinBody::Link::GEOMPROPERTIES::GeomBox:
                    if( xmlname == "extents" ) {
                        _ss >> _itgeomprop->vGeomData.x >> _itgeomprop->vGeomData.y >> _itgeomprop->vGeomData.z;
                    }
                    break;
                case KinBody::Link::GEOMPROPERTIES::GeomCylinder:
                    if( xmlname == "radius") {
                        _ss >> _itgeomprop->vGeomData.x;
                    }
                    else if( xmlname == "height" ) {
                        _ss >> _itgeomprop->vGeomData.y;
                    }
                    break;
                case KinBody::Link::GEOMPROPERTIES::GeomTrimesh:
                    if(( xmlname == "data") ||( xmlname == "collision") ) {
                        if( _collisionfilename.first.size() == 0 ) {
                            // check the attributes first
                            string orgfilename;
                            Vector vscale(1,1,1);
                            _ss >> orgfilename;
                            _ss >> vscale.x; vscale.y = vscale.z = vscale.x;
                            _ss >> vscale.y >> vscale.z;
                            _collisionfilename.first = !_fnGetModelsDir ? orgfilename : _fnGetModelsDir(orgfilename);
                            _collisionfilename.second = vscale;
                        }
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
                if( _masstype == MT_BoxMass ) {
                    _masstype = MT_Box;
                }
                else if( _masstype == MT_SphereMass) {
                    _masstype = MT_Sphere;
                }
                _ss >> _fMassDensity;
            }
            else if( xmlname == "total" ) {
                if( _masstype == MT_Box ) {
                    _masstype = MT_BoxMass;
                }
                else if( _masstype == MT_Sphere) {
                    _masstype = MT_SphereMass;
                }
                _ss >> _fTotalMass;
                _massCustom.fTotalMass = _fTotalMass;
            }
            else if( xmlname == "radius" ) {
                _ss >> _vMassExtents.x;
                _vMassExtents.x *= _vScaleGeometry.x;
            }
            else if((_masstype == MT_Box)&&(xmlname == "extents")) {
                _ss >> _vMassExtents.x >> _vMassExtents.y >> _vMassExtents.z;
                _vMassExtents *= _vScaleGeometry;
            }
            else if( xmlname == _processingtag ) {
                _processingtag = "";
            }
            else if( _masstype == MT_Custom ) {
                if( xmlname == "com" ) {
                    _ss >> _plink->_transMass.trans.x >> _plink->_transMass.trans.y >> _plink->_transMass.trans.z;
                    _massCustom.t.trans = _plink->_transMass.trans*_vScaleGeometry;
                }
                else if( xmlname == "inertia" ) {
                    _ss >> _massCustom.t.m[0] >> _massCustom.t.m[1] >> _massCustom.t.m[2] >> _massCustom.t.m[4] >> _massCustom.t.m[5] >> _massCustom.t.m[6] >> _massCustom.t.m[8] >> _massCustom.t.m[9] >> _massCustom.t.m[10];
                }
            }
            return false;
        }

        if( xmlname == "body" ) {
            if(( _plink->GetGeometries().size() == 0) && !_bSkipGeometry) {
                RAVELOG_WARN(str(boost::format("link %s has no geometry attached!\n")%_plink->GetName()));
            }
            // perform final processing stages
            MASS totalmass;
            if( _masstype == MT_MimicGeom ) {
                FOREACHC(itgeom, _plink->GetGeometries()) {
                    MASS mass;
                    switch(itgeom->GetType()) {
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
            else if( _masstype == MT_Box ) {
                totalmass = MASS::GetBoxMassD(_vMassExtents, Vector(), _fMassDensity);
            }
            else if( _masstype == MT_BoxMass ) {
                totalmass = MASS::GetBoxMass(_vMassExtents, Vector(), _fTotalMass);
            }
            else if( _masstype == MT_Sphere ) {
                totalmass = MASS::GetSphericalMassD(_vMassExtents.x, Vector(), _fMassDensity);
            }
            else if( _masstype == MT_Custom ) {
                totalmass = _massCustom;
            }
            else {
                totalmass = MASS::GetSphericalMass(_vMassExtents.x, Vector(), _fTotalMass);
            }
            _plink->_transMass = totalmass.t;
            _plink->_mass = totalmass.fTotalMass;
            tOrigTrans = _plink->GetTransform();

            Transform cur;
            if( !!_offsetfrom ) {
                // recompute new transformation
                Transform root;
                if( !!_fnGetOffsetFrom ) {
                    root = _fnGetOffsetFrom(_offsetfrom);
                }
                else {
                    root = _offsetfrom->GetTransform();
                }
                cur = _plink->GetTransform();
                cur = root * cur;
                tOrigTrans = root * tOrigTrans;         // update orig trans separately
                _plink->SetTransform(cur);
            }

            return true;
        }

        if( xmlname == "translation" ) {
            Vector v;
            _ss >> v.x >> v.y >> v.z;
            _plink->_t.trans += v*_vScaleGeometry;
        }
        else if( xmlname == "rotationmat" ) {
            TransformMatrix tnew;
            _ss >> tnew.m[0] >> tnew.m[1] >> tnew.m[2] >> tnew.m[4] >> tnew.m[5] >> tnew.m[6] >> tnew.m[8] >> tnew.m[9] >> tnew.m[10];
            _plink->_t.rot = (Transform(tnew)*_plink->_t).rot;
        }
        else if( xmlname == "rotationaxis" ) {
            Vector vaxis; dReal fangle=0;
            _ss >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
            Transform tnew; tnew.rot = quatFromAxisAngle(vaxis, fangle * PI / 180.0f);
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
                RAVELOG_WARN(str(boost::format("Failed to find offsetfrom body %s\n")%linkname));
                GetXMLErrorCount()++;
            }
        }

        if( xmlname !=_processingtag )
            RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%xmlname%_processingtag));
        _processingtag = "";
        return false;
    }

    virtual void characters(const std::string& ch)
    {
        if( _processingtag.size() > 0 ) {
            _ss.clear();
            _ss << ch;
        }
        else {
            StreamXMLReader::characters(ch);
        }
    }

    void SetMassType(MassType type, float fValue, const Vector& vMassExtents)
    {
        _masstype = type;
        _fMassDensity = _fTotalMass = fValue;
        _vMassExtents = vMassExtents;
    }

    Transform GetOrigTransform() const {
        return tOrigTrans;
    }

    boost::function<string(const std::string&)> _fnGetModelsDir;
    boost::function<Transform(KinBody::LinkPtr)> _fnGetOffsetFrom;

private:
    MASS _mass;                            ///< current mass of the object
    KinBody::LinkPtr& _plink;
    KinBodyPtr _pparent;
    KinBody::LinkPtr _offsetfrom;                            ///< all transformations are relative to the this body
    list<KinBody::Link::GEOMPROPERTIES>::iterator _itgeomprop;
    std::pair< string, Vector > _renderfilename, _collisionfilename;
    bool _bSkipGeometry;
    Vector _vScaleGeometry;
    Transform tOrigTrans;
    // Mass
    MassType _masstype;                   ///< if true, mass is craeted so that it mimics the geometry
    string _processingtag;         /// if not empty, currently processing
    MASS _massCustom;
    float _fMassDensity, _fTotalMass;
    Vector _vMassExtents;                   ///< used only if mass is a box
};

bool CreateGeometries(EnvironmentBasePtr penv, const std::string& filename, const Vector& vscale, std::list<KinBody::Link::GEOMPROPERTIES>& listGeometries)
{
    return LinkXMLReader::CreateGeometries(penv,filename,vscale,listGeometries);
}

// Joint Reader
class JointXMLReader : public StreamXMLReader
{
public:
    JointXMLReader(KinBody::JointPtr& pjoint, KinBodyPtr pparent, const AttributesList &atts) : _pjoint(pjoint) {
        _bNegateJoint = false;
        _pparent = pparent;
        _pjoint.reset(new KinBody::Joint(pparent));
        _pjoint->_type = KinBody::Joint::JointHinge;
        _vScaleGeometry = Vector(1,1,1);

        FOREACHC(itatt,atts) {
            if( itatt->first == "name" ) {
                _pjoint->_name = itatt->second;
            }
            else if( itatt->first == "type" ) {
                if( stricmp(itatt->second.c_str(), "hinge") == 0 ) {
                    _pjoint->_type = KinBody::Joint::JointHinge;
                }
                else if( stricmp(itatt->second.c_str(), "slider") == 0 ) {
                    _pjoint->_type = KinBody::Joint::JointSlider;
                }
                else if( stricmp(itatt->second.c_str(), "universal") == 0 ) {
                    _pjoint->_type = KinBody::Joint::JointUniversal;
                }
                else if( stricmp(itatt->second.c_str(), "hinge2") == 0 ) {
                    _pjoint->_type = KinBody::Joint::JointHinge2;
                }
                else if( stricmp(itatt->second.c_str(), "spherical") == 0 ) {
                    _pjoint->_type = KinBody::Joint::JointSpherical;
                }
                else {
                    RAVELOG_WARN(str(boost::format("unrecognized joint type: %s, setting to hinge\n")%itatt->second));
                    _pjoint->_type = KinBody::Joint::JointHinge;
                }
            }
            else if( itatt->first == "enable" ) {
                _pjoint->_bActive = !(stricmp(itatt->second.c_str(), "false") == 0 || itatt->second=="0");
            }
            else if( itatt->first == "mimic" ) {
                RAVELOG_WARN("mimic attribute on <joint> tag is deprecated! Use mimic_pos, mimic_vel, and mimic_accel\n");
                stringstream ss(itatt->second);
                dReal a=1, b=0;
                string strmimicjoint;
                ss >> strmimicjoint >> a >> b;
                if( !ss ) {
                    RAVELOG_WARN(str(boost::format("failed to set mimic properties correctly from: %s\n")%itatt->second));
                }
                _pjoint->_vmimic[0].reset(new KinBody::Joint::MIMIC());
                _pjoint->_vmimic[0]->_equations[0] = str(boost::format("%s*%f+%f")%strmimicjoint%a%b);
                _pjoint->_vmimic[0]->_equations[1] = str(boost::format("|%s %f")%strmimicjoint%a);
            }
            else if((itatt->first.size() >= 9)&&(itatt->first.substr(0,9) == "mimic_pos")) {
                if( !_pjoint->_vmimic[0] ) {
                    _pjoint->_vmimic[0].reset(new KinBody::Joint::MIMIC());
                }
                _pjoint->_vmimic[0]->_equations[0] = itatt->second;
            }
            else if((itatt->first.size() >= 9)&&(itatt->first.substr(0,9) == "mimic_vel")) {
                if( !_pjoint->_vmimic[0] ) {
                    _pjoint->_vmimic[0].reset(new KinBody::Joint::MIMIC());
                }
                _pjoint->_vmimic[0]->_equations[1] = itatt->second;
            }
            else if((itatt->first.size() >= 11)&&(itatt->first.substr(0,11) == "mimic_accel")) {
                if( !_pjoint->_vmimic[0] ) {
                    _pjoint->_vmimic[0].reset(new KinBody::Joint::MIMIC());
                }
                _pjoint->_vmimic[0]->_equations[2] = itatt->second;
            }
            else if( itatt->first == "circular" ) {
                _pjoint->_bIsCircular[0] = !(stricmp(itatt->second.c_str(), "false") == 0 || itatt->second=="0");
                for(int i = 1; i < _pjoint->GetDOF(); ++i) {
                    _pjoint->_bIsCircular[i] = _pjoint->_bIsCircular[0];
                }
            }
            else if( itatt->first == "scalegeometry" ) {
                stringstream ss(itatt->second);
                Vector v(1,1,1);
                ss >> v.x >> v.y >> v.z;
                if( !ss ) {
                    v.z = v.y = v.x;
                }
                _vScaleGeometry *= v;
            }
        }

        _vAxes.resize(_pjoint->GetDOF());
        if( _pjoint->GetType() == KinBody::Joint::JointSlider ) {
            for(int i = 0; i < _pjoint->GetDOF(); ++i) {
                _pjoint->_vlowerlimit.at(i) = -100000;
                _pjoint->_vupperlimit.at(i) = 100000;
            }
        }
        else if( _pjoint->GetType() == KinBody::Joint::JointSpherical ) {
            _vAxes.at(0) = Vector(1,0,0);
            _vAxes.at(1) = Vector(0,1,0);
            _vAxes.at(2) = Vector(0,0,1);
            for(int i = 0; i < _pjoint->GetDOF(); ++i) {
                _pjoint->_vlowerlimit.at(i) = -1000;
                _pjoint->_vupperlimit.at(i) = 1000;
            }
        }
        else {
            for(int i = 0; i < _pjoint->GetDOF(); ++i) {
                _pjoint->_vlowerlimit.at(i) = -PI;
                _pjoint->_vupperlimit.at(i) = PI;
            }
        }
        FOREACH(it,_pjoint->_vweights) {
            *it = 1;
        }
    }

    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList& atts)
    {
        switch( StreamXMLReader::startElement(xmlname,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }

        if (( xmlname == "body") ||( xmlname == "offsetfrom") ||( xmlname == "weight") ||( xmlname == "lostop") ||( xmlname == "histop") ||( xmlname == "limits") ||( xmlname == "limitsrad") ||( xmlname == "limitsdeg") ||( xmlname == "maxvel") || xmlname == "maxveldeg" ||( xmlname == "hardmaxvel") ||( xmlname == "maxaccel") || xmlname == "maxacceldeg" ||( xmlname == "maxtorque") ||( xmlname == "maxforce") ||( xmlname == "resolution") ||( xmlname == "anchor") ||( xmlname == "axis") ||( xmlname == "axis1") ||( xmlname == "axis2") ||( xmlname=="axis3") ||( xmlname == "mode") ||( xmlname == "initial") ) {
            _processingtag = xmlname;
            return PE_Support;
        }
        return PE_Pass;
    }

    virtual bool endElement(const std::string& xmlname)
    {
        int numindices = _pjoint->GetDOF();
        dReal fRatio = _pjoint->_type == KinBody::Joint::JointSlider ? (dReal)1 : (dReal)PI / 180.0f;         // most, but not all, joint take degrees

        if( !!_pcurreader ) {
            if( _pcurreader->endElement(xmlname) ) {
                _pcurreader.reset();
            }
        }
        else if( xmlname == "joint" ) {
            if( _pparent->GetLinks().size() == 0 ) {
                RAVELOG_WARN("parent kinbody has no links defined yet!\n");
                return false;
            }

            if( _bNegateJoint ) {
                FOREACH(itaxis,_vAxes) {
                    *itaxis = -*itaxis;
                }
            }

            string defaultname = "J_";
            // check if joint needs an artificial offset, only for revolute joints that have identifying points!
            if(( _pjoint->_type == KinBody::Joint::JointUniversal) ||( _pjoint->_type == KinBody::Joint::JointHinge2) ||( _pjoint->_type == KinBody::Joint::JointHinge) ) {
                for(int i = 0; i < numindices; ++i) {
                    if(( _pjoint->_vlowerlimit[i] < -PI) ||( _pjoint->_vupperlimit[i] > PI) ) {
                        _pjoint->_voffsets[i] = 0.5f * (_pjoint->_vlowerlimit[i] + _pjoint->_vupperlimit[i]);
                        if( _pjoint->_vupperlimit[i] - _pjoint->_voffsets[i] > PI ) {
                            RAVELOG_WARN(str(boost::format("joint %s, cannot allow joint ranges of more than 360 degrees\n")%_pjoint->GetName()));
                            _pjoint->_vupperlimit[i] = _pjoint->_voffsets[i] + PI - 1e-5;
                            _pjoint->_vlowerlimit[i] = _pjoint->_voffsets[i] - PI + 1e-5;
                        }
                    }
                }
            }

            Transform toffsetfrom;
            if( !!_offsetfrom ) {
                if( !!_fnGetOffsetFrom ) {
                    toffsetfrom = _fnGetOffsetFrom(_offsetfrom);
                }
                else {
                    toffsetfrom = _offsetfrom->GetTransform();
                }
            }

            toffsetfrom = attachedbodies[0]->GetTransform().inverse() * toffsetfrom;
            FOREACH(itaxis,_vAxes) {
                *itaxis = toffsetfrom.rotate(*itaxis);
            }
            _pjoint->_ComputeInternalInformation(attachedbodies[0],attachedbodies[1],toffsetfrom*_vanchor,_vAxes,_vinitialvalues);
            return true;
        }
        else if( xmlname == "weight" ) {
            for(int i = 0; i < numindices; ++i) {
                _ss >> _pjoint->_vweights.at(i);
            }
        }
        else if( xmlname == "initial" ) {
            _vinitialvalues = std::vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        }
        else if( xmlname == "body" ) {
            // figure out which body
            int index = !attachedbodies[0] ? 0 : 1;
            bool bQuery = true;
            string linkname;
            _ss >> linkname;

            FOREACHC(itlink, _pparent->GetLinks()) {
                if( stricmp((*itlink)->GetName().c_str(), linkname.c_str()) == 0 ) {
                    bQuery = !(*itlink)->IsStatic();
                    attachedbodies[index] = *itlink;
                    break;
                }
            }

            if( !attachedbodies[index] && bQuery ) {
                RAVELOG_WARN(str(boost::format("Failed to find body %s for joint %s\n")%linkname%_pjoint->_name));
                GetXMLErrorCount()++;
            }
        }
        else if((xmlname == "limits")||(xmlname == "limitsrad")||(xmlname == "limitsdeg")) {
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
            for(int i = 0; i < numindices; ++i) {
                _ss >> _pjoint->_vlowerlimit.at(i);
                _pjoint->_vlowerlimit.at(i) *= fRatio;
            }
        }
        else if( xmlname == "histop" ) {
            _bNegateJoint = true;
            RAVELOG_ERROR(str(boost::format("%s: <histop> deprecated, please use <limits> (now in radians), <limitsrad>, <limitsdeg> tag and negate your joint axis!\n")%_pparent->GetName()));
            for(int i = 0; i < numindices; ++i) {
                _ss >> _pjoint->_vupperlimit.at(i);
                _pjoint->_vupperlimit.at(i) *= fRatio;
            }
        }
        else if( xmlname == "maxvel" ) {
            for(int idof = 0; idof < _pjoint->GetDOF(); ++idof) {
                _ss >> _pjoint->_vmaxvel[idof];
            }
        }
        else if( xmlname == "maxveldeg" ) {
            for(int idof = 0; idof < _pjoint->GetDOF(); ++idof) {
                _ss >> _pjoint->_vmaxvel[idof];
                _pjoint->_vmaxvel[idof] *= PI/180.0;
            }
        }
        else if( xmlname == "hardmaxvel" ) {
            for(int idof = 0; idof < _pjoint->GetDOF(); ++idof) {
                _ss >> _pjoint->fHardMaxVel[idof];
            }
        }
        else if( xmlname == "maxaccel" ) {
            for(int idof = 0; idof < _pjoint->GetDOF(); ++idof) {
                _ss >> _pjoint->_vmaxaccel[idof];
            }
        }
        else if( xmlname == "maxacceldeg" ) {
            for(int idof = 0; idof < _pjoint->GetDOF(); ++idof) {
                _ss >> _pjoint->_vmaxaccel[idof];
                _pjoint->_vmaxaccel[idof] *= PI/180.0;
            }
        }
        else if( xmlname == "maxtorque" ) {
            for(int idof = 0; idof < _pjoint->GetDOF(); ++idof) {
                _ss >> _pjoint->_vmaxtorque[idof];
            }
        }
        else if( xmlname == "resolution" ) {
            _ss >> _pjoint->fResolution;
            _pjoint->fResolution *= fRatio;
        }
        else if( xmlname == "offsetfrom" ) {
            // figure out which body
            string linkname; _ss >> linkname;
            _offsetfrom = _pparent->GetLink(linkname);

            if( !_offsetfrom ) {
                RAVELOG_WARN(str(boost::format("Failed to find body %s\n")%linkname));
                GetXMLErrorCount()++;
            }
        }
        else {
            // could be type specific
            switch(_pjoint->_type) {
            case KinBody::Joint::JointHinge:
                if( xmlname == "anchor" ) {
                    _ss >> _vanchor.x >> _vanchor.y >> _vanchor.z;
                    _vanchor *= _vScaleGeometry;
                }
                else if( xmlname == "axis" ) {
                    _ss >> _vAxes.at(0).x >> _vAxes.at(0).y >> _vAxes.at(0).z;
                    _vAxes.at(0).normalize3();
                }
                break;
            case KinBody::Joint::JointSlider:
                if( xmlname == "axis" ) {
                    _ss >> _vAxes.at(0).x >> _vAxes.at(0).y >> _vAxes.at(0).z;
                    _vAxes.at(0).normalize3();
                }
                break;
            case KinBody::Joint::JointUniversal:
                if( xmlname == "anchor" ) {
                    _ss >> _vanchor.x >> _vanchor.y >> _vanchor.z;
                    _vanchor *= _vScaleGeometry;
                }
                else if( xmlname == "axis1" ) {
                    _ss >> _vAxes.at(0).x >> _vAxes.at(0).y >> _vAxes.at(0).z;
                    _vAxes.at(0).normalize3();
                }
                else if( xmlname == "axis2" ) {
                    _ss >> _vAxes.at(1).x >> _vAxes.at(1).y >> _vAxes.at(1).z;
                    _vAxes.at(0).normalize3();
                }
                break;
            case KinBody::Joint::JointHinge2:
                if( xmlname == "anchor" ) {
                    _ss >> _vanchor.x >> _vanchor.y >> _vanchor.z;
                    _vanchor *= _vScaleGeometry;
                }
                else if( xmlname == "axis1" ) {
                    _ss >> _vAxes.at(0).x >> _vAxes.at(0).y >> _vAxes.at(0).z;
                    _vAxes.at(0).normalize3();
                }
                else if( xmlname == "axis2" ) {
                    _ss >> _vAxes.at(1).x >> _vAxes.at(1).y >> _vAxes.at(1).z;
                    _vAxes.at(0).normalize3();
                }
                break;
            case KinBody::Joint::JointSpherical:
                if( xmlname == "anchor" ) {
                    _ss >> _vanchor.x >> _vanchor.y >> _vanchor.z;
                    _vanchor *= _vScaleGeometry;
                }
                break;
            default:
                throw openrave_exception(str(boost::format("bad joint type: 0x%x")%_pjoint->_type));
                break;
            }
        }

        _processingtag.resize(0);
        return false;
    }

    virtual void characters(const std::string& ch)
    {
        if( _processingtag.size() > 0 ) {
            _ss.clear();
            _ss << ch;
        }
        else {
            StreamXMLReader::characters(ch);
        }
    }

    boost::function<string(const std::string&)> _fnGetModelsDir;
    boost::function<Transform(KinBody::LinkPtr)> _fnGetOffsetFrom;

private:
    KinBody::LinkPtr _offsetfrom;         ///< all transforms are relative to this body
    KinBodyPtr _pparent;
    KinBody::JointPtr& _pjoint;
    std::vector<Vector> _vAxes;
    Vector _vanchor, _vScaleGeometry;
    bool _bNegateJoint;
    string _processingtag;
    boost::array<KinBody::LinkPtr,2> attachedbodies;
    std::vector<dReal> _vinitialvalues;
};

class InterfaceXMLReader;
typedef boost::shared_ptr<InterfaceXMLReader> InterfaceXMLReaderPtr;
typedef boost::shared_ptr<InterfaceXMLReader const> InterfaceXMLReaderConstPtr;

class InterfaceXMLReader : public StreamXMLReader
{
public:
    InterfaceXMLReader(EnvironmentBasePtr penv, InterfaceBasePtr& pinterface, InterfaceType type, const string &xmltag, const AttributesList &atts) : _penv(penv), _type(type), _pinterface(pinterface), _xmltag(xmltag) {
        _bProcessedLastTag = false;
        _atts = atts;
        string strtype;
        FOREACHC(itatt,atts) {
            if( itatt->first == "type" ) {
                strtype = itatt->second;
            }
            else if( itatt->first == "file" ) {
                AttributesList listnewatts;
                FOREACHC(itatt2,atts) {
                    if( itatt2->first != "file" ) {
                        listnewatts.push_back(*itatt2);
                    }
                }

                //BaseXMLReaderPtr preader = CreateInterfaceReader(_penv,_type,_pinterface, xmltag, listnewatts);
                //bool bSuccess = ParseXMLFile(preader, itatt->second);
                boost::shared_ptr<pair<string,string> > filedata = FindFile(itatt->second);
                if( !filedata ) {
                    continue;
                }

                string olddir = GetParseDirectory();
                string oldfile = GetFullFilename();
                try {
                    GetParseDirectory() = filedata->first;
                    GetFullFilename() = filedata->second;
                    if( !_pinterface ) {
                        // reason to bring all the other attributes since interface is not created yet? (there might be a problem with this?)
                        switch(_type) {
                        case PT_KinBody:
                            _pinterface = _penv->ReadKinBodyURI(KinBodyPtr(), filedata->second, listnewatts);
                            break;
                        case PT_Robot:
                            _pinterface = _penv->ReadRobotURI(RobotBasePtr(), filedata->second, listnewatts);
                            break;
                        default:
                            _pinterface = _penv->ReadInterfaceURI(filedata->second, listnewatts);
                        }
                        if( !!_pinterface &&( _pinterface->GetInterfaceType() != _type) ) {
                            RAVELOG_ERROR(str(boost::format("unexpected interface created %s\n")%RaveGetInterfaceName(_pinterface->GetInterfaceType())));
                            _pinterface.reset();
                        }
                    }
                    else {
                        switch(_type) {
                        case PT_KinBody:
                            _pinterface = _penv->ReadKinBodyURI(RaveInterfaceCast<KinBody>(_pinterface),filedata->second,listnewatts);
                            break;
                        case PT_Robot:
                            _pinterface = _penv->ReadRobotURI(RaveInterfaceCast<RobotBase>(_pinterface),filedata->second,listnewatts);
                            break;
                        default:
                            _pinterface = _penv->ReadInterfaceURI(_pinterface,_type,filedata->second,listnewatts);
                        }

                    }
                }
                catch(...) {
                    RAVELOG_ERROR(str(boost::format("failed to process %s\n")%itatt->second));
                    _pinterface.reset();
                }
                GetParseDirectory() = olddir;
                GetFullFilename() = oldfile;

                if( !_pinterface ) {
                    RAVELOG_DEBUG(str(boost::format("Failed to load filename %s\n")%itatt->second));
                    GetXMLErrorCount()++;
                    break;
                }
                _filename = _pinterface->GetURI();
            }
        }

        if( _xmltag.size() == 0 ) {
            _xmltag = RaveGetInterfaceName(_type);
        }

        if( strtype.size() > 0 ) {
            _pinterface = RaveCreateInterface(_penv, _type,strtype);
            if( !_pinterface ) {
                RAVELOG_ERROR(str(boost::format("xml readers failed to create instance of type %s:%s\n")%RaveGetInterfaceName(_type)%strtype));
                GetXMLErrorCount()++;
            }
            else {
                _pcustomreader = RaveCallXMLReader(_pinterface->GetInterfaceType(),_pinterface->GetXMLId(),_pinterface,_atts);
                SetFilename(_filename);
            }
        }
    }

    virtual void _CheckInterface()
    {
        if( !_pinterface ) {
            if( !_pinterface ) {
                switch(_type) {
                case PT_KinBody:
                    _pinterface = RaveCreateKinBody(_penv);
                    break;
                case PT_Robot:
                    _pinterface = RaveCreateInterface(_penv, PT_Robot, "GenericRobot");
                    if( !_pinterface ) {
                        _pinterface = RaveCreateInterface(_penv, PT_Robot, "");
                    }
                    break;
                case PT_Controller:
                    _pinterface = RaveCreateInterface(_penv, PT_Controller, "IdealController");
                    break;
                default:
                    _pinterface = RaveCreateInterface(_penv, _type, "");
                    break;
                }
            }

            if( !_pinterface ) {
                RAVELOG_ERROR(str(boost::format("xml readers failed to create instance of type %ss\n")%RaveGetInterfaceName(_type)));
            }
            else {
                _pcustomreader = RaveCallXMLReader(_pinterface->GetInterfaceType(),_pinterface->GetXMLId(),_pinterface,_atts);
            }

            SetFilename(_filename);
        }
    }

    void SetFilename(const string& filename)
    {
        if( !!_pinterface &&( _pinterface->__struri.size() == 0) ) {
            _pinterface->__struri = filename;
        }
    }

    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList& atts)
    {
        if( !!_pcustomreader ) {
            return _pcustomreader->startElement(xmlname, atts);
        }

        switch( StreamXMLReader::startElement(xmlname,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }

        // check for registers readers
        if( !!_pinterface ) {
            _pcustomreader = RaveCallXMLReader(_type,xmlname,_pinterface,atts);
            if( !!_pcustomreader ) {
                _readername = xmlname;
                if( !!_pcustomreader ) {
                    return PE_Support;
                }
            }
        }

        if (xmlname == "sendcommand" ) {
            _interfaceprocessingtag = xmlname;
            return PE_Support;
        }

        return PE_Pass;
    }

    virtual bool endElement(const std::string& xmlname)
    {
        if( !!_pcurreader ) {
            if( _pcurreader->endElement(xmlname) ) {
                _pcurreader.reset();
            }
        }
        else if( !!_pcustomreader ) {
            if( _pcustomreader->endElement(xmlname) ) {
                _CheckInterface();
                if( _readername.size() > 0 ) {
                    _pinterface->__mapReadableInterfaces[_readername] = _pcustomreader->GetReadable();
                }
                _pcustomreader.reset();
                if( xmlname == _xmltag ) {
                    return true;
                }
            }
        }
        else if( _interfaceprocessingtag.size() > 0 ) {
            if( xmlname == "sendcommand" ) {
                _CheckInterface();
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
            _interfaceprocessingtag.resize(0);
        }
        else if( xmlname == _xmltag ) {
            _CheckInterface();
            if( _bProcessedLastTag ) {
                RAVELOG_WARN(str(boost::format("already processed last tag for %s!\n")%xmlname));
            }
            _bProcessedLastTag = true;
            return true;
        }
        return false;
    }

    virtual void characters(const std::string& ch)
    {
        if( _interfaceprocessingtag.size() > 0 ) {
            _ss.clear();
            _ss << ch;
        }
        else if( !!_pcustomreader ) {
            _pcustomreader->characters(ch);
        }
        else {
            StreamXMLReader::characters(ch);
        }
    }

    virtual XMLReadablePtr GetReadable() {
        return XMLReadablePtr(new InterfaceXMLReadable(_pinterface));
    }
protected:
    EnvironmentBasePtr _penv;
    InterfaceType _type;
    InterfaceBasePtr& _pinterface;
    BaseXMLReaderPtr _pcustomreader;
    string _xmltag, _interfaceprocessingtag;
    string _interfacename, _readername;
    bool _bProcessedLastTag;
    AttributesList _atts;
};
/// KinBody reader
/// reads kinematic chain specific entries, can instantiate this reader from another reader
class KinBodyXMLReader : public InterfaceXMLReader
{
public:
    KinBodyXMLReader(EnvironmentBasePtr penv, InterfaceBasePtr& pchain, InterfaceType type, const AttributesList &atts, int roottransoffset) : InterfaceXMLReader(penv,pchain,type,"kinbody",atts), roottransoffset(roottransoffset) {
        _bSkipGeometry = false;
        _vScaleGeometry = Vector(1,1,1);
        _masstype = LinkXMLReader::MT_None;
        _fMassValue = 1;
        _vMassExtents = Vector(1,1,1);
        _bOverwriteDiffuse = false;
        _bOverwriteAmbient = false;
        _bOverwriteTransparency = false;
        _bMakeJoinedLinksAdjacent = true;
        rootoffset = rootjoffset = rootjpoffset = -1;
        FOREACHC(itatt,atts) {
            if( itatt->first == "prefix" ) {
                _prefix = itatt->second;
            }
            else if( itatt->first == "name" ) {
                _bodyname = itatt->second;
            }
            else if( itatt->first == "makejoinedlinksadjacent") {
                _bMakeJoinedLinksAdjacent = atoi(itatt->second.c_str())!=0;
            }
            else if( itatt->first == "skipgeometry" ) {
                _bSkipGeometry = stricmp(itatt->second.c_str(), "true") == 0 || itatt->second=="1";
            }
            else if( itatt->first == "scalegeometry" ) {
                stringstream ss(itatt->second);
                Vector v(1,1,1);
                ss >> v.x >> v.y >> v.z;
                if( !ss ) {
                    v.z = v.y = v.x;
                }
                _vScaleGeometry *= v;
            }
            else if((itatt->first != "file")&&(itatt->first != "type")) {
                RAVELOG_WARN(str(boost::format("unknown kinbody attribute %s\n")%itatt->first));
            }
        }
        _CheckInterface();
    }

    virtual void _CheckInterface()
    {
        InterfaceXMLReader::_CheckInterface();
        _pchain = RaveInterfaceCast<KinBody>(_pinterface);
        if( !!_pchain &&( rootoffset < 0) ) {
            _pchain->_bMakeJoinedLinksAdjacent = _bMakeJoinedLinksAdjacent;
            rootoffset = (int)_pchain->GetLinks().size();
            rootjoffset = (int)_pchain->GetJoints().size();
            rootjpoffset = (int)_pchain->GetPassiveJoints().size();
            //RAVELOG_INFO(str(boost::format("links: %d, prefix: %s: %x\n")%_pchain->GetLinks().size()%_prefix%this));
            // reisze _vTransforms to be the same size as the initial number of links
            _pchain->GetLinkTransformations(_vTransforms);
            _pchain->SetViewerData(UserDataPtr());
        }
    }

    Transform GetOffsetFrom(KinBody::LinkPtr plink)
    {
        if(( plink->GetIndex() < 0) ||( plink->GetIndex() >= (int)_vTransforms.size()) ) {
            return plink->GetTransform();
        }
        return _vTransforms[plink->GetIndex()];
    }

    string GetModelsDir(const std::string& filename) const
    {
        if( filename.size() == 0 ) {
            return filename;
        }
#ifdef _WIN32
        if( filename.find_first_of(':') != string::npos ) {
            return filename;
        }
#else
        if(( filename[0] == '/') ||( filename[0] == '~') ) {
            return filename;
        }
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
            if( !!ifstream(temp.c_str()) ) {
                return temp;
            }
            FOREACHC(itdir, RaveGetDataDirs()) {
                temp = *itdir; temp.push_back(s_filesep);
                temp += *itmodelsdir;
                temp += filename;
                if( !!ifstream(temp.c_str()) ) {
                    return temp;
                }
            }
        }

        return "";         // bad filename
    }

    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList& atts)
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
            AttributesList newatts = atts;
            newatts.push_back(make_pair("skipgeometry",_bSkipGeometry ? "1" : "0"));
            newatts.push_back(make_pair("scalegeometry",str(boost::format("%f %f %f")%_vScaleGeometry.x%_vScaleGeometry.y%_vScaleGeometry.z)));
            _pcurreader = CreateInterfaceReader(_penv,PT_KinBody,_pinterface, xmlname, newatts);
            return PE_Support;
        }

        _CheckInterface();
        if( xmlname == "body" ) {
            _plink.reset();
            AttributesList newatts = atts;
            newatts.push_back(make_pair("skipgeometry",_bSkipGeometry ? "1" : "0"));
            newatts.push_back(make_pair("scalegeometry",str(boost::format("%f %f %f")%_vScaleGeometry.x%_vScaleGeometry.y%_vScaleGeometry.z)));
            boost::shared_ptr<LinkXMLReader> plinkreader(new LinkXMLReader(_plink, _pchain, newatts));
            plinkreader->SetMassType(_masstype, _fMassValue, _vMassExtents);
            plinkreader->_fnGetModelsDir = boost::bind(&KinBodyXMLReader::GetModelsDir,this,_1);
            plinkreader->_fnGetOffsetFrom = boost::bind(&KinBodyXMLReader::GetOffsetFrom,this,_1);
            _pcurreader = plinkreader;
            return PE_Support;
        }
        else if( xmlname == "joint" ) {
            _pjoint.reset();
            AttributesList newatts = atts;
            newatts.push_back(make_pair("scalegeometry",str(boost::format("%f %f %f")%_vScaleGeometry.x%_vScaleGeometry.y%_vScaleGeometry.z)));
            boost::shared_ptr<JointXMLReader> pjointreader(new JointXMLReader(_pjoint,_pchain, atts));
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

        static boost::array<string, 10> tags = { { "translation", "rotationmat", "rotationaxis", "quat", "jointvalues", "adjacent", "modelsdir", "diffusecolor", "transparency", "ambientcolor"}};
        if( find(tags.begin(),tags.end(),xmlname) != tags.end() ) {
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

                    if( _plink->_index < 0 ) {
                        // not in array yet
                        _plink->_index = (int)_pchain->_veclinks.size();
                        _pchain->_veclinks.push_back(_plink);
                        _vTransforms.push_back(Transform());
                    }

                    // do this later, or else offsetfrom will be messed up!
                    _vTransforms[_plink->GetIndex()] = boost::dynamic_pointer_cast<LinkXMLReader>(_pcurreader)->GetOrigTransform();
                    _plink.reset();
                }
                else if( xmlname == "joint" ) {
                    _pjoint->dofindex = _pchain->GetDOF();
                    boost::shared_ptr<JointXMLReader> pjointreader = boost::dynamic_pointer_cast<JointXMLReader>(_pcurreader);
                    if( _pjoint->_bActive ) {
                        _pjoint->jointindex = (int)_pchain->_vecjoints.size();
                        _pchain->_vecjoints.push_back(_pjoint);
                    }
                    else {
                        _pjoint->jointindex = -1;
                        _pjoint->dofindex = -1;
                        _pchain->_vPassiveJoints.push_back(_pjoint);
                    }
                    BOOST_ASSERT( _pjoint->dofindex < _pchain->GetDOF());
                    _pjoint.reset();
                }
                else if( xmlname == "kinbody" ) {
                    // most likely new transforms were added, so update
                    _CheckInterface();
                    _pchain->GetLinkTransformations(_vTransforms);
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
            else if((_masstype == LinkXMLReader::MT_Box)&&(xmlname == "extents")) {
                _ss >> _vMassExtents.x >> _vMassExtents.y >> _vMassExtents.z;
            }
        }
        else if( _processingtag.size() > 0 ) {
            if( xmlname == "translation" ) {
                Vector v;
                _ss >> v.x >> v.y >> v.z;
                _trans.trans += v*_vScaleGeometry;
            }
            else if( xmlname == "rotationaxis" ) {
                Vector vaxis; dReal fangle=0;
                _ss >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
                Transform tnew; tnew.rot = quatFromAxisAngle(vaxis, fangle * PI / 180.0f);
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
                getline(_ss,_strModelsDir);
                boost::trim(_strModelsDir);
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
                _vjointvalues.reset(new std::vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>()));
            }

            if( xmlname !=_processingtag ) {
                RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%xmlname%_processingtag));
            }
            _processingtag = "";
        }
        else if( InterfaceXMLReader::endElement(xmlname) ) {
            if( _bodyname.size() > 0 ) {
                _pchain->SetName(_bodyname);
            }
            if( _filename.size() > 0 ) {
                SetFilename(_filename);
            }

            // add prefix
            if( _prefix.size() > 0 ) {
                //RAVELOG_INFO("write prefix: links: %d-%d, 0x%x\n",rootoffset,(int)_pchain->_veclinks.size(),this);
                BOOST_ASSERT(rootoffset >= 0 && rootoffset<=(int)_pchain->_veclinks.size());
                for(vector<KinBody::LinkPtr>::iterator itlink = _pchain->_veclinks.begin()+rootoffset; itlink != _pchain->_veclinks.end(); ++itlink) {
                    (*itlink)->_name = _prefix + (*itlink)->_name;
                }
                BOOST_ASSERT(rootjoffset >= 0 && rootjoffset<=(int)_pchain->_vecjoints.size());
                for(vector<KinBody::JointPtr>::iterator itjoint = _pchain->_vecjoints.begin()+rootjoffset; itjoint != _pchain->_vecjoints.end(); ++itjoint) {
                    (*itjoint)->_name = _prefix +(*itjoint)->_name;
                }
                BOOST_ASSERT(rootjpoffset >= 0 && rootjpoffset<=(int)_pchain->_vPassiveJoints.size());
                for(vector<KinBody::JointPtr>::iterator itjoint = _pchain->_vPassiveJoints.begin()+rootjpoffset; itjoint != _pchain->_vPassiveJoints.end(); ++itjoint) {
                    (*itjoint)->_name = _prefix +(*itjoint)->_name;
                }
            }

            if( _bOverwriteDiffuse ) {
                // overwrite the color
                FOREACH(itlink, _pchain->_veclinks) {
                    FOREACH(itprop, (*itlink)->_listGeomProperties) {
                        itprop->diffuseColor = _diffusecol;
                    }
                }
            }
            if( _bOverwriteAmbient ) {
                // overwrite the color
                FOREACH(itlink, _pchain->_veclinks) {
                    FOREACH(itprop, (*itlink)->_listGeomProperties) {
                        itprop->ambientColor = _ambientcol;
                    }
                }
            }
            if( _bOverwriteTransparency ) {
                // overwrite the color
                FOREACH(itlink, _pchain->_veclinks) {
                    FOREACH(itprop, (*itlink)->_listGeomProperties) {
                        itprop->ftransparency = _transparency;
                    }
                }
            }

            // transform all the bodies with trans
            Transform cur;
            BOOST_ASSERT(roottransoffset>=0 && roottransoffset<=(int)_pchain->_veclinks.size());
            for(vector<KinBody::LinkPtr>::iterator itlink = _pchain->_veclinks.begin()+roottransoffset; itlink != _pchain->_veclinks.end(); ++itlink) {
                (*itlink)->SetTransform(_trans * (*itlink)->GetTransform());
            }
            Vector com = _pchain->GetCenterOfMass();
            RAVELOG_VERBOSE("%s: COM = (%f,%f,%f)\n", _pchain->GetName().c_str(), com.x, com.y, com.z);
            return true;
        }
        return false;
    }

    virtual void characters(const std::string& ch)
    {
        if( _processingtag.size() > 0 ) {
            _ss.clear();
            _ss << ch;
        }
        else {
            InterfaceXMLReader::characters(ch);
        }
    }

    const boost::shared_ptr< std::vector<dReal> > GetJointValues() {
        return _vjointvalues;
    }
protected:
    KinBodyPtr _pchain;
    Transform _trans;

    // default mass type passed to every LinkXMLReader
    int rootoffset, rootjoffset, rootjpoffset, roottransoffset;                         ///< the initial number of links when KinBody is created (so that global translations and rotations only affect the new links)
    LinkXMLReader::MassType _masstype;                     ///< if true, mass is craeted so that it mimics the geometry
    float _fMassValue;                       ///< density or total mass
    Vector _vMassExtents;

    vector<Transform> _vTransforms;             ///< original transforms of the bodies for offsetfrom

    string _strModelsDir, _bodyname;
    string _prefix;         ///< add this prefix to all names of links and joints
    KinBody::LinkPtr _plink;
    KinBody::JointPtr _pjoint;

    RaveVector<float> _diffusecol, _ambientcol;
    float _transparency;
    bool _bSkipGeometry;
    Vector _vScaleGeometry;
    bool _bMakeJoinedLinksAdjacent;
    boost::shared_ptr< std::vector<dReal> > _vjointvalues;

    string _processingtag;         /// if not empty, currently processing
    bool _bOverwriteDiffuse, _bOverwriteAmbient, _bOverwriteTransparency;
};

class ControllerXMLReader : public InterfaceXMLReader
{
public:
    ControllerXMLReader(EnvironmentBasePtr penv, InterfaceBasePtr& pinterface, const AttributesList &atts,RobotBasePtr probot=RobotBasePtr()) : InterfaceXMLReader(penv,pinterface,PT_Controller,RaveGetInterfaceName(PT_Controller),atts) {
        _probot = probot;
        nControlTransformation = 0;
        FOREACHC(itatt, atts) {
            if( itatt->first == "robot" ) {
                _robotname = itatt->second;
            }
            else if( itatt->first == "joints" ) {
                stringstream ss(itatt->second);
                _vjoints.reset(new std::vector<std::string>((istream_iterator<std::string>(ss)), istream_iterator<std::string>()));
            }
            else if( itatt->first == "transform" ) {
                stringstream ss(itatt->second);
                ss >> nControlTransformation;
            }
        }
    }
    virtual ~ControllerXMLReader() {
    }

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

            if( !!_probot ) {
                std::vector<int> dofindices;
                if( !_vjoints ) {
                    for(int i = 0; i < _probot->GetDOF(); ++i) {
                        dofindices.push_back(i);
                    }
                }
                else {
                    FOREACH(it,*_vjoints) {
                        KinBody::JointPtr pjoint = _probot->GetJoint(*it);
                        if( !!pjoint ) {
                            for(int i = 0; i < pjoint->GetDOF(); ++i) {
                                dofindices.push_back(pjoint->GetDOFIndex()+i);
                            }
                        }
                        else {
                            RAVELOG_WARN(str(boost::format("could not find joint %s\n")%*it));
                        }
                    }
                }
                _CheckInterface();
                _probot->SetController(RaveInterfaceCast<ControllerBase>(_pinterface),dofindices,nControlTransformation);
            }
            else {
                RAVELOG_WARN("controller is unused\n");
            }
            return true;
        }
        return false;
    }

    string _robotname;
    boost::shared_ptr< vector<string> > _vjoints;
    int nControlTransformation;
    RobotBasePtr _probot;
};

class ManipulatorXMLReader : public StreamXMLReader
{
public:
    ManipulatorXMLReader(RobotBase::ManipulatorPtr& pmanip, RobotBasePtr probot, const AttributesList &atts) : _pmanip(pmanip) {
        _vScaleGeometry = Vector(1,1,1);
        if( !_pmanip ) {
            _pmanip.reset(new RobotBase::Manipulator(probot));
        }
        FOREACHC(itatt,atts) {
            if( itatt->first == "name" ) {
                _pmanip->_name = itatt->second;
            }
        }

        _probot = _pmanip->GetRobot();
    }

    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList& atts)
    {
        switch( StreamXMLReader::startElement(xmlname,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }

        if( _processingtag.size() > 0 )
            return PE_Ignore;

        if (( xmlname == "effector") ||( xmlname == "gripperjoints") ||( xmlname == "joints") ||( xmlname == "armjoints") ||( xmlname == "base") ||( xmlname == "iksolver") ||( xmlname == "closingdir") ||( xmlname == "palmdirection") ||( xmlname=="direction") ||( xmlname == "closingdirection") ||( xmlname == "translation") ||( xmlname == "quat") ||( xmlname == "rotationaxis") ||( xmlname == "rotationmat") ) {
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
            return true;
        }
        else if( xmlname == "effector" ) {
            // look up the correct link
            string linkname; _ss >> linkname;
            _pmanip->_pEndEffector = _probot->GetLink(linkname);

            if( !_pmanip->_pEndEffector ) {
                RAVELOG_WARN("Failed to find manipulator end effector %s\n", linkname.c_str());
                GetXMLErrorCount()++;
            }
        }
        else if( xmlname == "base" ) {
            string linkname; _ss >> linkname;
            _pmanip->_pBase = _probot->GetLink(linkname);
            if( !_pmanip->_pBase ) {
                RAVELOG_WARN("Failed to find manipulator base %s\n", linkname.c_str());
                GetXMLErrorCount()++;
            }
        }
        else if((xmlname == "joints")||(xmlname == "gripperjoints")) {
            _pmanip->_vgripperjointnames = vector<string>((istream_iterator<string>(_ss)), istream_iterator<string>());
        }
        else if( xmlname == "armjoints" ) {
            RAVELOG_WARN("<armjoints> for <manipulator> tag is not used anymore\n");
        }
        else if((xmlname == "direction")||(xmlname == "palmdirection")) {
            if( xmlname == "palmdirection" )
                RAVELOG_WARN("<palmdirection> tag in Manipulator changed to <direction>\n");
            _ss >> _pmanip->_vdirection.x >> _pmanip->_vdirection.y >> _pmanip->_vdirection.z;
            dReal flen = _pmanip->_vdirection.lengthsqr3();
            if( flen == 0 ) {
                RAVELOG_WARN("palm direction is 0, setting to default value\n");
                _pmanip->_vdirection = Vector(0,0,1);
            }
            else
                _pmanip->_vdirection /= RaveSqrt(flen);
        }
        else if( xmlname == "iksolver" ) {
            string iklibraryname = _ss.str();

            IkSolverBasePtr piksolver;
            if( RaveHasInterface(PT_IkSolver,iklibraryname) ) {
                piksolver = RaveCreateIkSolver(_probot->GetEnv(), iklibraryname);
            }
            if( !piksolver ) {
                // try adding the current directory
                if( RaveHasInterface(PT_IkSolver,GetParseDirectory()+s_filesep+iklibraryname)) {
                    string fullname = GetParseDirectory(); fullname.push_back(s_filesep); fullname += iklibraryname;
                    piksolver = RaveCreateIkSolver(_probot->GetEnv(), fullname);
                }

                if( !piksolver ) {
                    // try loading the shared object
                    ModuleBasePtr pIKFastLoader;
                    {
                        list<ModuleBasePtr> listModules;
                        boost::shared_ptr<void> pmutex = _probot->GetEnv()->GetModules(listModules);
                        FOREACHC(itprob, listModules) {
                            if( stricmp((*itprob)->GetXMLId().c_str(),"ikfast") == 0 ) {
                                pIKFastLoader = *itprob;
                                break;
                            }
                        }
                    }

                    if( !pIKFastLoader ) {
                        pIKFastLoader = RaveCreateModule(_probot->GetEnv(), "ikfast");
                        if( !!pIKFastLoader )
                            _probot->GetEnv()->AddModule(pIKFastLoader,"");
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
                                piksolver = RaveCreateIkSolver(_probot->GetEnv(), fullname);
                            }
                        }
                        else {
                            string fullname = "ikfast "; fullname += iklibraryname;
                            piksolver = RaveCreateIkSolver(_probot->GetEnv(), fullname);
                        }
                    }
                    else {
                        RAVELOG_WARN("Failed to load IKFast module\n");
                    }
                }
            }

            if( !piksolver )
                RAVELOG_WARN("failed to create iksolver %s\n", iklibraryname.c_str());
            else {
                _pmanip->_strIkSolver = piksolver->GetXMLId();
            }

            _pmanip->_pIkSolver = piksolver;
            if( !!piksolver ) {
                _pmanip->_strIkSolver = piksolver->GetXMLId();
            }
        }
        else if((xmlname == "closingdirection")||(xmlname == "closingdir")) {
            _pmanip->_vClosingDirection = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
            FOREACH(it, _pmanip->_vClosingDirection) {
                if( *it > 0 ) {
                    *it = 1;
                }
                else if( *it < 0 ) {
                    *it = -1;
                }
            }
        }
        else if( xmlname == "translation" ) {
            Vector v;
            _ss >> v.x >> v.y >> v.z;
            _pmanip->_tLocalTool.trans += v*_vScaleGeometry;
        }
        else if( xmlname == "quat" ) {
            Transform tnew;
            _ss >> tnew.rot.x >> tnew.rot.y >> tnew.rot.z >> tnew.rot.w;
            tnew.rot.normalize4();
            _pmanip->_tLocalTool.rot = (tnew*_pmanip->_tLocalTool).rot;
        }
        else if( xmlname == "rotationaxis" ) {
            Vector vaxis; dReal fangle=0;
            _ss >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
            Transform tnew; tnew.rot = quatFromAxisAngle(vaxis, fangle * PI / 180.0f);
            _pmanip->_tLocalTool.rot = (tnew*_pmanip->_tLocalTool).rot;
        }
        else if( xmlname == "rotationmat" ) {
            TransformMatrix tnew;
            _ss >> tnew.m[0] >> tnew.m[1] >> tnew.m[2] >> tnew.m[4] >> tnew.m[5] >> tnew.m[6] >> tnew.m[8] >> tnew.m[9] >> tnew.m[10];
            _pmanip->_tLocalTool.rot = (Transform(tnew)*_pmanip->_tLocalTool).rot;
        }

        if( xmlname !=_processingtag )
            RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%xmlname%_processingtag));
        _processingtag = "";
        return false;
    }

    virtual void characters(const std::string& ch)
    {
        if( _processingtag.size() > 0 ) {
            _ss.clear();
            _ss << ch;
        }
        else {
            StreamXMLReader::characters(ch);
        }
    }

protected:
    RobotBasePtr _probot;
    RobotBase::ManipulatorPtr& _pmanip;
    string _processingtag;
    Vector _vScaleGeometry;
};

/// sensors specifically attached to a robot
class AttachedSensorXMLReader : public StreamXMLReader
{
public:
    AttachedSensorXMLReader(RobotBase::AttachedSensorPtr& psensor, RobotBasePtr probot, const AttributesList &atts) : _psensor(psensor) {
        string name;
        _vScaleGeometry = Vector(1,1,1);
        FOREACHC(itatt, atts) {
            if( itatt->first == "name" ) {
                name = itatt->second;
            }
        }

        if( !_psensor ) {
            // check for current sensors
            FOREACH(itsensor,probot->GetAttachedSensors()) {
                if(( name.size() > 0) &&( (*itsensor)->GetName() == name) ) {
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

    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList& atts)
    {
        switch( StreamXMLReader::startElement(xmlname,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }

        if( _processingtag.size() > 0 ) {
            return PE_Ignore;
        }
        if( xmlname == "sensor" ) {
            // create the sensor
            _psensorinterface.reset();
            _pcurreader = CreateInterfaceReader(_probot->GetEnv(),PT_Sensor,_psensorinterface, xmlname, atts);
            return PE_Support;
        }

        if (( xmlname == "link") ||( xmlname == "translation") ||( xmlname == "quat") ||( xmlname == "rotationaxis") ||( xmlname == "rotationmat") ) {
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
                RAVELOG_VERBOSE("Attached robot sensor %s points to no real sensor!\n",_psensor->GetName().c_str());
            }
            else {
                _psensor->pdata = _psensor->psensor->CreateSensorData();
                if( _psensor->pattachedlink.expired() ) {
                    RAVELOG_INFOA("no attached link, setting to base of robot\n");
                    if( _probot->GetLinks().size() == 0 ) {
                        RAVELOG_INFOA("robot has no links!\n");
                        _psensor->pattachedlink.reset();
                    }
                    else {
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
                RAVELOG_WARN("Failed to find attached sensor link %s\n", linkname.c_str());
                GetXMLErrorCount()++;
            }
        }
        else if( xmlname == "translation" ) {
            Vector v;
            _ss >> v.x >> v.y >> v.z;
            _psensor->trelative.trans += v*_vScaleGeometry;
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
            Transform tnew; tnew.rot = quatFromAxisAngle(vaxis, fangle * PI / 180.0f);
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

    virtual void characters(const std::string& ch)
    {
        if( _processingtag.size() > 0 ) {
            _ss.clear();
            _ss << ch;
        }
        else {
            StreamXMLReader::characters(ch);
        }
    }

protected:
    RobotBasePtr _probot;
    RobotBase::AttachedSensorPtr& _psensor;
    InterfaceBasePtr _psensorinterface;
    string _processingtag;
    string args;         ///< arguments to pass to sensor when initializing
    Vector _vScaleGeometry;
};

class RobotXMLReader : public InterfaceXMLReader
{
public:
    RobotXMLReader(EnvironmentBasePtr penv, InterfaceBasePtr& probot, const AttributesList &atts, int roottransoffset) : InterfaceXMLReader(penv,probot,PT_Robot,"robot",atts), roottransoffset(roottransoffset) {
        _bSkipGeometry = false;
        _vScaleGeometry = Vector(1,1,1);
        rootoffset = rootjoffset = rootjpoffset = -1;
        FOREACHC(itatt, atts) {
            if( itatt->first == "name" ) {
                _robotname = itatt->second;
            }
            else if( itatt->first == "prefix" ) {
                _prefix = itatt->second;
            }
            else if( itatt->first == "skipgeometry" ) {
                _bSkipGeometry = stricmp(itatt->second.c_str(), "true") == 0 || itatt->second=="1";
            }
            else if( itatt->first == "scalegeometry" ) {
                stringstream ss(itatt->second);
                Vector v(1,1,1);
                ss >> v.x >> v.y >> v.z;
                if( !ss ) {
                    v.z = v.y = v.x;
                }
                _vScaleGeometry *= v;
            }
        }
        _CheckInterface();
    }

    virtual void _CheckInterface()
    {
        InterfaceXMLReader::_CheckInterface();
        _probot = RaveInterfaceCast<RobotBase>(_pinterface);
        if( !!_probot ) {
            if( rootoffset < 0 ) {
                rootoffset = (int)_probot->GetLinks().size();
                rootjoffset = (int)_probot->GetJoints().size();
                rootjpoffset = (int)_probot->GetPassiveJoints().size();
                FOREACH(itmanip,_probot->GetManipulators()) {
                    _setInitialManipulators.insert(*itmanip);
                }
                FOREACH(itsensor,_probot->GetAttachedSensors()) {
                    _setInitialSensors.insert(*itsensor);
                }
            }
        }
    }

    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList& atts)
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
            AttributesList newatts = atts;
            newatts.push_back(make_pair("skipgeometry",_bSkipGeometry ? "1" : "0"));
            newatts.push_back(make_pair("scalegeometry",str(boost::format("%f %f %f")%_vScaleGeometry.x%_vScaleGeometry.y%_vScaleGeometry.z)));
            _pcurreader = CreateInterfaceReader(_penv, PT_Robot, _pinterface, xmlname, newatts);
            return PE_Support;
        }

        _CheckInterface();
        if( xmlname == "kinbody" ) {
            AttributesList newatts = atts;
            newatts.push_back(make_pair("skipgeometry",_bSkipGeometry ? "1" : "0"));
            newatts.push_back(make_pair("scalegeometry",str(boost::format("%f %f %f")%_vScaleGeometry.x%_vScaleGeometry.y%_vScaleGeometry.z)));
            _pcurreader = CreateInterfaceReader(_penv,PT_KinBody,_pinterface, xmlname, newatts);
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
        else if((xmlname == "translation")||(xmlname == "rotationmat")||(xmlname == "rotationaxis")||(xmlname == "quat")||(xmlname == "jointvalues")) {
            _processingtag = xmlname;
        }
        else {
            return PE_Pass;
        }
        return PE_Support;
    }

    virtual bool endElement(const std::string& xmlname)
    {
        if( !!_pcurreader ) {
            if( _pcurreader->endElement(xmlname) ) {
                KinBodyXMLReaderPtr kinbodyreader = boost::dynamic_pointer_cast<KinBodyXMLReader>(_pcurreader);
                if( !!kinbodyreader ) {
                    if( !_vjointvalues ) {
                        _vjointvalues = kinbodyreader->GetJointValues();
                    }
                }
                _pcurreader.reset();
            }
            _probot = RaveInterfaceCast<RobotBase>(_pinterface);         // might be updated by readers
            return false;
        }
        else if( _processingtag.size() > 0 ) {
            if( xmlname == "translation" ) {
                Vector v;
                _ss >> v.x >> v.y >> v.z;
                _trans.trans += v*_vScaleGeometry;
            }
            else if( xmlname == "rotationaxis" ) {
                Vector vaxis; dReal fangle=0;
                _ss >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
                Transform tnew; tnew.rot = quatFromAxisAngle(vaxis, fangle * PI / 180.0f);
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
                _vjointvalues.reset(new std::vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>()));
            }

            if( xmlname !=_processingtag ) {
                RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%xmlname%_processingtag));
            }
            _processingtag = "";
        }
        else if( InterfaceXMLReader::endElement(xmlname) ) {
            if( _robotname.size() > 0 ) {
                _probot->SetName(_robotname);
            }
            if( _filename.size() > 0 ) {
                SetFilename(_filename);
            }

            // put the sensors and manipulators in front of what was declared. this is necessary so that user-based manipulator definitions come before the pre-defined ones.
            if( _setInitialSensors.size() > 0 ) {
                std::vector<RobotBase::AttachedSensorPtr> vtemp; vtemp.reserve(_probot->GetAttachedSensors().size());
                FOREACH(itsensor,_probot->GetAttachedSensors()) {
                    if( _setInitialSensors.find(*itsensor) == _setInitialSensors.end() ) {
                        vtemp.insert(vtemp.begin(),*itsensor);
                    }
                    else {
                        vtemp.push_back(*itsensor);
                    }
                }
                _probot->GetAttachedSensors().swap(vtemp);
            }
            if( _setInitialManipulators.size() > 0 ) {
                std::vector<RobotBase::ManipulatorPtr> vtemp; vtemp.reserve(_probot->GetManipulators().size());
                FOREACH(itmanip,_probot->GetManipulators()) {
                    if( _setInitialManipulators.find(*itmanip) == _setInitialManipulators.end() ) {
                        vtemp.insert(vtemp.begin(),*itmanip);
                    }
                    else {
                        vtemp.push_back(*itmanip);
                    }
                }
                _probot->GetManipulators().swap(vtemp);
            }

            // add prefix
            if( _prefix.size() > 0 ) {
                BOOST_ASSERT(rootoffset >= 0 && rootoffset<=(int)_probot->_veclinks.size());
                vector<KinBody::LinkPtr>::iterator itlink = _probot->_veclinks.begin()+rootoffset;
                while(itlink != _probot->_veclinks.end()) {
                    (*itlink)->_name = _prefix + (*itlink)->_name;
                    ++itlink;
                }
                std::vector< std::pair<std::string, std::string> > jointnamepairs;
                jointnamepairs.reserve(_probot->_vecjoints.size());
                BOOST_ASSERT(rootjoffset >= 0 && rootjoffset<=(int)_probot->_vecjoints.size());
                vector<KinBody::JointPtr>::iterator itjoint = _probot->_vecjoints.begin()+rootjoffset;
                list<KinBody::JointPtr> listjoints;
                while(itjoint != _probot->_vecjoints.end()) {
                    jointnamepairs.push_back(make_pair((*itjoint)->_name, _prefix +(*itjoint)->_name));
                    (*itjoint)->_name = _prefix +(*itjoint)->_name;
                    listjoints.push_back(*itjoint);
                    ++itjoint;
                }
                BOOST_ASSERT(rootjpoffset >= 0 && rootjpoffset<=(int)_probot->_vPassiveJoints.size());
                itjoint = _probot->_vPassiveJoints.begin()+rootjpoffset;
                while(itjoint != _probot->_vPassiveJoints.end()) {
                    jointnamepairs.push_back(make_pair((*itjoint)->_name, _prefix +(*itjoint)->_name));
                    (*itjoint)->_name = _prefix +(*itjoint)->_name;
                    listjoints.push_back(*itjoint);
                    ++itjoint;
                }
                // repeat again for the mimic equations, if any exist
                FOREACH(itjoint2, listjoints) {
                    for(int idof = 0; idof < (*itjoint2)->GetDOF(); ++idof) {
                        if( (*itjoint2)->IsMimic(idof) ) {
                            for(int ieq = 0; ieq < 3; ++ieq) {
                                string neweq;
                                SearchAndReplace(neweq,(*itjoint2)->_vmimic[idof]->_equations[ieq],jointnamepairs);
                                (*itjoint2)->_vmimic[idof]->_equations[ieq] = neweq;
                            }
                        }
                    }
                }
                FOREACH(itsensor, _probot->GetAttachedSensors()) {
                    if( _setInitialSensors.find(*itsensor) == _setInitialSensors.end() ) {
                        (*itsensor)->_name = _prefix + (*itsensor)->_name;
                    }
                }
                FOREACH(itmanip,_probot->GetManipulators()) {
                    if( _setInitialManipulators.find(*itmanip) == _setInitialManipulators.end()) {
                        (*itmanip)->_name = _prefix + (*itmanip)->_name;
                        FOREACH(itgrippername,(*itmanip)->_vgripperjointnames) {
                            *itgrippername = _prefix + *itgrippername;
                        }
                    }
                }
            }

            // transform all "new" bodies with trans
            BOOST_ASSERT(roottransoffset>=0&&roottransoffset<=(int)_probot->_veclinks.size());
            vector<KinBody::LinkPtr>::iterator itlink = _probot->_veclinks.begin()+roottransoffset;
            while(itlink != _probot->_veclinks.end()) {
                (*itlink)->SetTransform(_trans * (*itlink)->GetTransform());
                ++itlink;
            }

            // forces robot to reupdate its internal objects
            _probot->SetTransform(_probot->GetTransform());
            return true;
        }
        return false;
    }

    virtual void characters(const std::string& ch)
    {
        if( _processingtag.size() > 0 ) {
            _ss.clear();
            _ss << ch;
        }
        else {
            InterfaceXMLReader::characters(ch);
        }
    }

    const boost::shared_ptr< std::vector<dReal> > GetJointValues() {
        return _vjointvalues;
    }

protected:
    RobotBasePtr _probot;
    InterfaceBasePtr _pcontroller;         ///< controller to set the robot at
    string _robotname;
    string _prefix;
    string _processingtag;

    boost::shared_ptr<std::vector<dReal> >  _vjointvalues;
    RobotBase::AttachedSensorPtr _psensor;

    Transform _trans;
    bool _bSkipGeometry;
    Vector _vScaleGeometry;
    int rootoffset, roottransoffset;                         ///< the initial number of links when Robot is created (so that global translations and rotations only affect the new links)
    int rootjoffset, rootjpoffset;         ///< the initial number of joints when Robot is created
    std::set<RobotBase::ManipulatorPtr> _setInitialManipulators;
    std::set<RobotBase::AttachedSensorPtr> _setInitialSensors;
};

template <InterfaceType type> class DummyInterfaceXMLReader : public InterfaceXMLReader
{
public:
    DummyInterfaceXMLReader(EnvironmentBasePtr penv, InterfaceBasePtr& pinterface, const string &xmltag, const AttributesList &atts) : InterfaceXMLReader(penv,pinterface,type,xmltag,atts) {
    }
    virtual ~DummyInterfaceXMLReader() {
    }
};

class ModuleXMLReader : public InterfaceXMLReader
{
public:
    ModuleXMLReader(EnvironmentBasePtr penv, InterfaceBasePtr& pinterface, const AttributesList &atts) : InterfaceXMLReader(penv,pinterface,PT_Module,RaveGetInterfaceName(PT_Module),atts) {
        FOREACHC(itatt,atts) {
            if( itatt->first == "args" ) {
                _args = itatt->second;
            }
        }

        _CheckInterface();
        if( !!_pinterface ) {
            ModuleBasePtr module = RaveInterfaceCast<ModuleBase>(_pinterface);
            if( !!module ) {
                int ret = _penv->AddModule(module,_args);
                if( ret ) {
                    RAVELOG_WARN(str(boost::format("module %s returned %d\n")%module->GetXMLId()%ret));
                    module.reset();
                    _pinterface.reset();
                }
            }
        }
    }

    string _args;
};

class SensorXMLReader : public InterfaceXMLReader
{
public:
    SensorXMLReader(EnvironmentBasePtr penv, InterfaceBasePtr& pinterface, const AttributesList &atts) : InterfaceXMLReader(penv,pinterface,PT_Sensor,RaveGetInterfaceName(PT_Sensor),atts) {
        string args;
        _vScaleGeometry = Vector(1,1,1);
        FOREACHC(itatt,atts) {
            if( itatt->first == "name" ) {
                _strname = itatt->second;
            }
            else if( itatt->first == "args" ) {
                RAVELOG_WARN("sensor args has been deprecated.\n");
            }
        }
    }

    virtual void _CheckInterface()
    {
        InterfaceXMLReader::_CheckInterface();
        _psensor = RaveInterfaceCast<SensorBase>(_pinterface);
        if( !!_psensor ) {
            _psensor->SetName(_strname);
        }
    }

    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList& atts)
    {
        if( _processingtag.size() > 0 ) {
            switch( StreamXMLReader::startElement(xmlname,atts) ) {
            case PE_Pass: break;
            case PE_Support: return PE_Support;
            case PE_Ignore: return PE_Ignore;
            }
        }

        switch( InterfaceXMLReader::startElement(xmlname,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }

        _CheckInterface();
        if( !_psensor ||( _processingtag.size() > 0) ) {
            return PE_Ignore;
        }
        if (( xmlname == "translation") ||( xmlname == "quat") ||( xmlname == "rotationaxis") ||( xmlname == "rotationmat") ) {
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
            }
            return false;
        }
        else if( _processingtag.size() > 0 ) {
            if( xmlname == "translation" ) {
                Vector v;
                _ss >> v.x >> v.y >> v.z;
                _tsensor.trans += v*_vScaleGeometry;
            }
            else if( xmlname == "quat" ) {
                Transform tnew;
                _ss >> tnew.rot.x >> tnew.rot.y >> tnew.rot.z >> tnew.rot.w;
                tnew.rot.normalize4();
                _tsensor.rot = (tnew*_tsensor).rot;
            }
            else if( xmlname == "rotationaxis" ) {
                Vector vaxis; dReal fangle=0;
                _ss >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
                Transform tnew; tnew.rot = quatFromAxisAngle(vaxis, fangle * PI / 180.0f);
                _tsensor.rot = (tnew*_tsensor).rot;
            }
            else if( xmlname == "rotationmat" ) {
                TransformMatrix tnew;
                _ss >> tnew.m[0] >> tnew.m[1] >> tnew.m[2] >> tnew.m[4] >> tnew.m[5] >> tnew.m[6] >> tnew.m[8] >> tnew.m[9] >> tnew.m[10];
                _tsensor.rot = (Transform(tnew)*_tsensor).rot;
            }

            if( xmlname !=_processingtag ) {
                RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%xmlname%_processingtag));
            }
            _processingtag.resize(0);
        }
        else if( InterfaceXMLReader::endElement(xmlname) ) {
            if( !!_psensor ) {
                _psensor->SetTransform(_tsensor);
            }
            return true;
        }
        return false;
    }

    virtual void characters(const std::string& ch)
    {
        if( _processingtag.size() > 0 ) {
            _ss.clear();
            _ss << ch;
        }
        else {
            InterfaceXMLReader::characters(ch);
        }
    }

    const string& GetArgs() const {
        return _args;
    }

protected:
    SensorBasePtr _psensor;
    Transform _tsensor;
    string _args, _processingtag, _strname;
    Vector _vScaleGeometry;
};

class EnvironmentXMLReader : public StreamXMLReader
{
public:
    EnvironmentXMLReader(EnvironmentBasePtr penv, const AttributesList &atts, bool bInEnvironment) : _penv(penv), _bInEnvironment(bInEnvironment)
    {
        if( !_penv ) {
            throw openrave_exception("need valid environment",ORE_InvalidArguments);
        }
        FOREACHC(itatt,atts) {
            if( itatt->first == "file" ) {
                AttributesList listnewatts;
                FOREACHC(itatt2,atts) {
                    if( itatt2->first != "file" ) {
                        listnewatts.push_back(*itatt2);
                    }
                }

                boost::shared_ptr<pair<string,string> > filedata = FindFile(itatt->second);
                if( !filedata ) {
                    continue;
                }
                _penv->Load(filedata->second);
            }
        }
        tCamera.trans = Vector(0, 1.5f, 0.8f);
        tCamera.rot = quatFromAxisAngle(Vector(1, 0, 0), (dReal)-0.5);
        vBkgndColor = Vector(1,1,1);
        bTransSpecified = false;
    }

    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList& atts)
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
                if( !!_pinterface ) {
                    throw openrave_exception("interface should not be initialized");
                }
                _pcurreader = CreateInterfaceReader(_penv,itname->first,_pinterface,"",atts);
                if( !_pcurreader ) {
                    RAVELOG_WARN("failed to create interface %s in <environment>\n", itname->second.c_str());
                    _pcurreader.reset(new DummyXMLReader(xmlname,"environment"));
                }
                return PE_Support;
            }
        }

        if( xmlname == "environment" ) {
            _pcurreader.reset(new EnvironmentXMLReader(_penv,atts,true));
            return PE_Support;
        }

        static boost::array<string, 7> tags = { { "bkgndcolor", "camrotaxis", "camrotationaxis", "camrotmat", "camtrans", "bkgndcolor", "plugin"}};
        if( find(tags.begin(),tags.end(),xmlname) != tags.end() ) {
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
                    if( !!pinterfacereader ) {
                        pinterfacereader->SetFilename(_filename);
                    }
                }

                if( !!boost::dynamic_pointer_cast<RobotXMLReader>(_pcurreader) ) {
                    boost::shared_ptr<RobotXMLReader> robotreader = boost::dynamic_pointer_cast<RobotXMLReader>(_pcurreader);
                    BOOST_ASSERT(_pinterface->GetInterfaceType()==PT_Robot);
                    RobotBasePtr probot = RaveInterfaceCast<RobotBase>(_pinterface);
                    _penv->AddRobot(probot);
                    if( !!robotreader->GetJointValues() ) {
                        if( (int)robotreader->GetJointValues()->size() != probot->GetDOF() ) {
                            RAVELOG_WARN(str(boost::format("<jointvalues> wrong number of values %d!=%d, robot=%s")%robotreader->GetJointValues()->size()%probot->GetDOF()%probot->GetName()));
                        }
                        else {
                            probot->SetDOFValues(*robotreader->GetJointValues());
                        }
                    }
                }
                else if( !!boost::dynamic_pointer_cast<KinBodyXMLReader>(_pcurreader) ) {
                    KinBodyXMLReaderPtr kinbodyreader = boost::dynamic_pointer_cast<KinBodyXMLReader>(_pcurreader);
                    BOOST_ASSERT(_pinterface->GetInterfaceType()==PT_KinBody);
                    KinBodyPtr pbody = RaveInterfaceCast<KinBody>(_pinterface);
                    _penv->AddKinBody(pbody);
                    if( !!kinbodyreader->GetJointValues() ) {
                        if( (int)kinbodyreader->GetJointValues()->size() != pbody->GetDOF() ) {
                            RAVELOG_WARN(str(boost::format("<jointvalues> wrong number of values %d!=%d, body=%s")%kinbodyreader->GetJointValues()->size()%pbody->GetDOF()%pbody->GetName()));
                        }
                        else {
                            pbody->SetDOFValues(*kinbodyreader->GetJointValues());
                        }
                    }
                }
                else if( !!boost::dynamic_pointer_cast<SensorXMLReader>(_pcurreader) ) {
                    BOOST_ASSERT(_pinterface->GetInterfaceType()==PT_Sensor);
                    _penv->AddSensor(RaveInterfaceCast<SensorBase>(_pinterface));
                }
                else if( !!boost::dynamic_pointer_cast< DummyInterfaceXMLReader<PT_PhysicsEngine> >(_pcurreader) ) {
                    BOOST_ASSERT(_pinterface->GetInterfaceType()==PT_PhysicsEngine);
                    _penv->SetPhysicsEngine(RaveInterfaceCast<PhysicsEngineBase>(_pinterface));
                }
                else if( !!boost::dynamic_pointer_cast< DummyInterfaceXMLReader<PT_CollisionChecker> >(_pcurreader) ) {
                    BOOST_ASSERT(_pinterface->GetInterfaceType()==PT_CollisionChecker);
                    _penv->SetCollisionChecker(RaveInterfaceCast<CollisionCheckerBase>(_pinterface));
                }
                else if( !!_pinterface ) {
                    RAVELOG_DEBUG("owning interface %s, type: %s\n",_pinterface->GetXMLId().c_str(),RaveGetInterfaceName(_pinterface->GetInterfaceType()).c_str());
                    _penv->OwnInterface(_pinterface);
                }
                _pinterface.reset();
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
        else if((xmlname == "camrotaxis")||(xmlname == "camrotationaxis")) {
            Vector vaxis; dReal fangle=0;
            _ss >> vaxis.x >> vaxis.y >> vaxis.z >> fangle;
            tCamera.rot = quatFromAxisAngle(vaxis, fangle * PI / 180.0f);
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
            RaveLoadPlugin(pluginname);
        }

        if( xmlname !=_processingtag ) {
            RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%xmlname%_processingtag));
        }
        _processingtag = "";
        return false;
    }

protected:
    EnvironmentBasePtr _penv;
    InterfaceBasePtr _pinterface;         // current processed interface
    Vector vBkgndColor;
    Transform tCamera;         ///< default camera transformationn
    string _processingtag;
    bool bTransSpecified;
    bool _bInEnvironment;
};

BaseXMLReaderPtr CreateEnvironmentReader(EnvironmentBasePtr penv, const AttributesList& atts)
{
    return BaseXMLReaderPtr(new EnvironmentXMLReader(penv,atts,false));
}

BaseXMLReaderPtr CreateInterfaceReader(EnvironmentBasePtr penv, InterfaceType type, InterfaceBasePtr& pinterface, const std::string& xmltag, const AttributesList& atts)
{
    switch(type) {
    case PT_Planner: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_Planner>(penv,pinterface,xmltag,atts));
    case PT_Robot: {
        KinBodyPtr pbody = RaveInterfaceCast<KinBody>(pinterface);
        int rootoffset = 0;
        if( !!pbody ) {
            rootoffset = (int)pbody->GetLinks().size();
        }
        return InterfaceXMLReaderPtr(new RobotXMLReader(penv,pinterface,atts,rootoffset));
    }
    case PT_SensorSystem: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_SensorSystem>(penv,pinterface,xmltag,atts));
    case PT_Controller: return InterfaceXMLReaderPtr(new ControllerXMLReader(penv,pinterface,atts));
    case PT_Module: return InterfaceXMLReaderPtr(new ModuleXMLReader(penv,pinterface,atts));
    case PT_IkSolver: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_IkSolver>(penv,pinterface,xmltag,atts));
    case PT_KinBody: {
        KinBodyPtr pbody = RaveInterfaceCast<KinBody>(pinterface);
        int rootoffset = 0;
        if( !!pbody ) {
            rootoffset = (int)pbody->GetLinks().size();
        }
        return InterfaceXMLReaderPtr(new KinBodyXMLReader(penv,pinterface,type,atts,rootoffset));
    }
    case PT_PhysicsEngine: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_PhysicsEngine>(penv,pinterface,xmltag,atts));
    case PT_Sensor: return InterfaceXMLReaderPtr(new SensorXMLReader(penv,pinterface,atts));
    case PT_CollisionChecker: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_CollisionChecker>(penv,pinterface,xmltag,atts));
    case PT_Trajectory: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_Trajectory>(penv,pinterface,xmltag,atts));
    case PT_Viewer: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_Viewer>(penv,pinterface,xmltag,atts));
    case PT_SpaceSampler: return InterfaceXMLReaderPtr(new DummyInterfaceXMLReader<PT_SpaceSampler>(penv,pinterface,xmltag,atts));
    }

    throw openrave_exception(str(boost::format("could not create interface of type %d")%type),ORE_InvalidArguments);
}

class GlobalInterfaceXMLReader : public StreamXMLReader
{
public:
    GlobalInterfaceXMLReader(EnvironmentBasePtr penv, const AttributesList &atts, bool bAddToEnvironment=false) : _penv(penv), _atts(atts), _bAddToEnvironment(bAddToEnvironment) {
    }
    virtual ProcessElement startElement(const std::string& xmlname, const AttributesList& atts)
    {
        switch( StreamXMLReader::startElement(xmlname,atts) ) {
        case PE_Pass: break;
        case PE_Support: return PE_Support;
        case PE_Ignore: return PE_Ignore;
        }
        AttributesList newatts = atts;
        newatts.insert(newatts.end(),_atts.begin(),_atts.end());
        _pinterface.reset();

        if( xmlname == "environment" ) {
            _pcurreader = CreateEnvironmentReader(_penv,newatts);
            if( !!_pcurreader ) {
                return PE_Support;
            }
        }

        // check for any plugins
        FOREACHC(itname,RaveGetInterfaceNamesMap()) {
            if( xmlname == itname->second ) {
                if( !!_pinterface ) {
                    throw openrave_exception("interface should not be initialized");
                }
                _pcurreader = CreateInterfaceReader(_penv,itname->first,_pinterface,"",newatts);
                if( !_pinterface ) {
                    throw openrave_exception(str(boost::format("failed to create interface %s")%itname->second));
                }
                return PE_Support;
            }
        }

        throw openrave_exception(str(boost::format("invalid interface tag %s")%xmlname));
    }

    virtual bool endElement(const std::string& xmlname)
    {
        if( !!_pcurreader ) {
            if( _pcurreader->endElement(xmlname) ) {
                if( !!_pinterface ) {
                    if( _bAddToEnvironment ) {
                        if( _pinterface->GetInterfaceType() == PT_Robot ) {
                            _penv->AddRobot(RaveInterfaceCast<RobotBase>(_pinterface));
                        }
                        else if( _pinterface->GetInterfaceType() == PT_KinBody ) {
                            _penv->AddKinBody(RaveInterfaceCast<KinBody>(_pinterface));
                        }
                        else if( _pinterface->GetInterfaceType() == PT_Sensor ) {
                            _penv->AddSensor(RaveInterfaceCast<SensorBase>(_pinterface));
                        }
//                        else if( _pinterface->GetInterfaceType() == PT_Module ) {
//                            _penv->AddModule(RaveInterfaceCast<ModuleBase>(_pinterface),"");
//                        }
                        else if( _pinterface->GetInterfaceType() == PT_Viewer ) {
                            _penv->AddViewer(RaveInterfaceCast<ViewerBase>(_pinterface));
                        }
                    }
                    return true;
                }
                bool bisenvironment = !!boost::dynamic_pointer_cast<EnvironmentXMLReader>(_pcurreader);
                _pcurreader.reset();
                return bisenvironment;
            }
        }
        return false;
    }

    virtual XMLReadablePtr GetReadable() {
        return XMLReadablePtr(new InterfaceXMLReadable(_pinterface));
    }
protected:
    EnvironmentBasePtr _penv;
    InterfaceBasePtr _pinterface;         // current processed interface
    AttributesList _atts;         ///< attributes to always set on newly created interfaces
    bool _bAddToEnvironment; ///< if true, will add interface to environment
};

BaseXMLReaderPtr CreateInterfaceReader(EnvironmentBasePtr penv, const AttributesList& atts,bool bAddToEnvironment)
{
    return BaseXMLReaderPtr(new OpenRAVEXMLParser::GlobalInterfaceXMLReader(penv,atts,bAddToEnvironment));
}

} // end namespace OpenRAVEXMLParser
