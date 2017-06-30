/*
 * Copyright 2006 Sony Computer Entertainment Inc.
 *
 * Licensed under the MIT Open Source License, for details please see license.txt or the website
 * http://www.opensource.org/licenses/mit-license.php
 *
 */

// The user can choose whether or not to include libxml support in the DOM. Supporting libxml will
// require linking against it. By default libxml support is included.
#if defined(DOM_INCLUDE_LIBXML)

// This is a rework of the XML plugin that contains a complete interface to libxml2 "readXML"
// This is intended to be a seperate plugin but I'm starting out by rewriting it in daeLIBXMLPlugin
// because I'm not sure if all the plugin handling stuff has been tested.  Once I get a working
// plugin I'll look into renaming it so the old daeLIBXMLPlugin can coexist with it.
//
#include <string>
#include <sstream>
#include <modules/daeLIBXMLPlugin.h>
#include <dae.h>
#include <dom.h>
#include <dae/daeDatabase.h>
#include <dae/daeMetaElement.h>
#include <libxml/xmlreader.h>
#include <libxml/xmlwriter.h>
#include <libxml/xmlmemory.h>
#include <dae/daeErrorHandler.h>
#include <dae/daeMetaElementAttribute.h>

#include <limits>
#include <iomanip>
using namespace std;

#include <zip.h> // for saving compressed files
#ifdef _WIN32
#include <iowin32.h>
#else
#include <unistd.h>
#endif

// Some helper functions for working with libxml
namespace {
daeInt getCurrentLineNumber(xmlTextReaderPtr reader) {
#if LIBXML_VERSION >= 20620
    return xmlTextReaderGetParserLineNumber(reader);
#else
    return -1;
#endif
}

#ifdef _WIN32
static const char s_filesep = '\\';
#else
static const char s_filesep = '/';
#endif

// Return value should be freed by caller with delete[]. Passed in value should not
// be null.
xmlChar* utf8ToLatin1(const xmlChar* utf8) {
    int inLen = xmlStrlen(utf8);
    int outLen = (inLen+1) * 2;
    xmlChar* latin1 = new xmlChar[outLen];
    int numBytes = UTF8Toisolat1(latin1, &outLen, utf8, &inLen);
    if (numBytes < 0) {
        // Failed. Return an empty string instead.
        numBytes = 0;
    }
    latin1[numBytes] = '\0';
    return latin1;
}

// Return value should be freed by caller with delete[].
xmlChar* latin1ToUtf8(const string& latin1) {
    int inLen = (int)latin1.length();
    int outLen = (inLen+1) * 2;
    xmlChar* utf8 = new xmlChar[outLen];
    int numBytes = isolat1ToUTF8(utf8, &outLen, (xmlChar*)latin1.c_str(), &inLen);
    if (numBytes < 0)
        // Failed. Return an empty string instead.
        numBytes = 0;

    utf8[numBytes] = '\0';
    return utf8;
}

// wrapper that automatically closes the zip file handle
class zipFileHandler
{
public:
    zipFileHandler() {
        zf = NULL;
    }
    ~zipFileHandler() {
        if( !!zf ) {
            int errclose = zipClose(zf,NULL);
            if (errclose != ZIP_OK) {
                ostringstream msg;
                msg << "zipClose error" << errclose << "\n";
                daeErrorHandler::get()->handleError(msg.str().c_str());
            }
        }
    }
    zipFile zf;
};

class xmlBufferHandler
{
public:
    xmlBufferHandler() {
        buf = NULL;
    }
    ~xmlBufferHandler() {
        if( !!buf ) {
            xmlBufferFree(buf);
        }
    }
    xmlBufferPtr buf;
};

typedef pair<daeString, daeString> stringPair;

// The attributes vector passed in should be empty. If 'encoding' is anything
// other than utf8 the caller should free the returned attribute value
// strings. The 'freeAttrValues' function is provided for that purpose.
void packageCurrentAttributes(xmlTextReaderPtr reader,
                              DAE::charEncoding encoding,
                              /* out */ vector<stringPair>& attributes) {
    int numAttributes = xmlTextReaderAttributeCount(reader);
    if (numAttributes == -1 || numAttributes == 0)
        return;
    attributes.reserve(numAttributes);

    while (xmlTextReaderMoveToNextAttribute(reader) == 1) {
        const xmlChar* xmlName = xmlTextReaderConstName(reader);
        const xmlChar* xmlValue = xmlTextReaderConstValue(reader);
        if (encoding == DAE::Latin1)
            attributes.push_back(stringPair((daeString)xmlName, (daeString)utf8ToLatin1(xmlValue)));
        else
            attributes.push_back(stringPair((daeString)xmlName, (daeString)xmlValue));
    }
}

void freeAttrValues(vector<stringPair>& pairs) {
    for(size_t i=0, size=pairs.size(); i<size; ++i) {
        delete[] pairs[i].second;
        pairs[i].second = 0;
    }
}
}

daeLIBXMLPlugin::daeLIBXMLPlugin(DAE& dae) : dae(dae), rawRelPath(dae)
{
    supportedProtocols.push_back("*");
    xmlInitParser();
    rawFile = NULL;
    rawByteCount = 0;
    saveRawFile = false;
    writer = NULL;
}

daeLIBXMLPlugin::~daeLIBXMLPlugin()
{
    if( !writer ) {
        xmlFreeTextWriter( writer );
        writer = NULL;
    }
    xmlCleanupParser();
}

daeInt daeLIBXMLPlugin::setOption( daeString option, daeString value )
{
    if ( strcmp( option, "saveRawBinary" ) == 0 )
    {
        if ( strcmp( value, "true" ) == 0 || strcmp( value, "TRUE" ) == 0 )
        {
            saveRawFile = true;
        }
        else
        {
            saveRawFile = false;
        }
        return DAE_OK;
    }
    return DAE_ERR_INVALID_CALL;
}

daeString daeLIBXMLPlugin::getOption( daeString option )
{
    if ( strcmp( option, "saveRawBinary" ) == 0 )
    {
        if ( saveRawFile )
        {
            return "true";
        }
        return "false";
    }
    return NULL;
}

namespace {
void libxmlErrorHandler(void* arg,
                        const char* msg,
                        xmlParserSeverities severity,
                        xmlTextReaderLocatorPtr locator) {
    if(severity == XML_PARSER_SEVERITY_VALIDITY_WARNING  ||
       severity == XML_PARSER_SEVERITY_WARNING) {
        daeErrorHandler::get()->handleWarning(msg);
    }
    else
        daeErrorHandler::get()->handleError(msg);
}
}

#if LIBXML_VERSION < 20700
#ifndef XML_PARSE_HUGE
#define XML_PARSE_HUGE 0
#endif
#endif

// A simple structure to help alloc/free xmlTextReader objects
struct xmlTextReaderHelper {
    xmlTextReaderHelper(const daeURI& uri) {
        if((reader = xmlReaderForFile(cdom::fixUriForLibxml(uri.str()).c_str(), NULL, XML_PARSE_HUGE)))
            xmlTextReaderSetErrorHandler(reader, libxmlErrorHandler, NULL);
    }

    xmlTextReaderHelper(daeString buffer, const daeURI& baseUri) {
        if((reader = xmlReaderForDoc((xmlChar*)buffer, cdom::fixUriForLibxml(baseUri.str()).c_str(), NULL, XML_PARSE_HUGE)))
            xmlTextReaderSetErrorHandler(reader, libxmlErrorHandler, NULL);
    };

    ~xmlTextReaderHelper() {
        if (reader)
            xmlFreeTextReader(reader);
    }

    xmlTextReaderPtr reader;
};

daeElementRef daeLIBXMLPlugin::readFromFile(const daeURI& uri) {
    xmlTextReaderHelper readerHelper(uri);
    if (!readerHelper.reader) {
        daeErrorHandler::get()->handleError((string("Failed to open ") + uri.str() +
                                             " in daeLIBXMLPlugin::readFromFile\n").c_str());
        return NULL;
    }
    return read(readerHelper.reader);
}

daeElementRef daeLIBXMLPlugin::readFromMemory(daeString buffer, const daeURI& baseUri) {
    xmlTextReaderHelper readerHelper(buffer, baseUri);
    if (!readerHelper.reader) {
        daeErrorHandler::get()->handleError("Failed to open XML document from memory buffer in "
                                            "daeLIBXMLPlugin::readFromMemory\n");
        return NULL;
    }
    return read(readerHelper.reader);
}

daeElementRef daeLIBXMLPlugin::read(_xmlTextReader* reader) {
    // Drop everything up to the first element. In the future, we should try to store header comments somewhere.
    while(xmlTextReaderNodeType(reader) != XML_READER_TYPE_ELEMENT)
    {
        if (xmlTextReaderRead(reader) != 1) {
            daeErrorHandler::get()->handleError("Error parsing XML in daeLIBXMLPlugin::read\n");
            return NULL;
        }
    }

    int readRetVal = 0;
    return readElement(reader, NULL, readRetVal);
}

daeElementRef daeLIBXMLPlugin::readElement(_xmlTextReader* reader,
                                           daeElement* parentElement,
                                           /* out */ int& readRetVal) {
    assert(xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT);
    daeString elementName = (daeString)xmlTextReaderConstName(reader);
    bool empty = xmlTextReaderIsEmptyElement(reader) != 0;

    vector<attrPair> attributes;
    packageCurrentAttributes(reader, dae.getCharEncoding(), /* out */ attributes);

    daeElementRef element = beginReadElement(parentElement, elementName, attributes, getCurrentLineNumber(reader));
    if (dae.getCharEncoding() != DAE::Utf8) {
        freeAttrValues(attributes);
    }

    if (!element) {
        // We couldn't create the element. beginReadElement already printed an error message. Just make sure
        // to skip ahead past the bad element.
        xmlTextReaderNext(reader);
        return NULL;
    }

    if ((readRetVal = xmlTextReaderRead(reader)) == -1) {
        return NULL;
    }
    if (empty) {
        return element;
    }
    int nodeType = xmlTextReaderNodeType(reader);
    while (readRetVal == 1  &&  nodeType != XML_READER_TYPE_END_ELEMENT) {
        if (nodeType == XML_READER_TYPE_ELEMENT) {
            element->placeElement(readElement(reader, element, readRetVal));
        }
        else if (nodeType == XML_READER_TYPE_TEXT) {
            const xmlChar* xmlText = xmlTextReaderConstValue(reader);
            if (dae.getCharEncoding() == DAE::Latin1) {
                xmlText = utf8ToLatin1(xmlText);
            }
            readElementText(element, (daeString)xmlText, getCurrentLineNumber(reader));
            if (dae.getCharEncoding() == DAE::Latin1) {
                delete[] xmlText;
            }
            readRetVal = xmlTextReaderRead(reader);
        }
        else {
            readRetVal = xmlTextReaderRead(reader);
        }
        nodeType = xmlTextReaderNodeType(reader);
    }

    if (nodeType == XML_READER_TYPE_END_ELEMENT) {
        readRetVal = xmlTextReaderRead(reader);
    }
    if (readRetVal == -1) { // Something went wrong (bad xml probably)
        return NULL;
    }

    return element;
}

daeInt daeLIBXMLPlugin::write(const daeURI& name, daeDocument *document, daeBool replace)
{
    // Make sure database and document are both set
    if (!database) {
        return DAE_ERR_INVALID_CALL;
    }
    if(!document) {
        return DAE_ERR_COLLECTION_DOES_NOT_EXIST;
    }
    // Convert the URI to a file path, to see if we're about to overwrite a file
    string file = cdom::uriToNativePath(name.str());
    if (file.empty()  &&  saveRawFile)
    {
        daeErrorHandler::get()->handleError( "can't get path in write\n" );
        return DAE_ERR_BACKEND_IO;
    }

    // If replace=false, don't replace existing files
    if(!replace)
    {
        // Using "stat" would be better, but it's not available on all platforms
        FILE *tempfd = fopen(file.c_str(), "r");
        if(tempfd != NULL)
        {
            // File exists, return error
            fclose(tempfd);
            return DAE_ERR_BACKEND_FILE_EXISTS;
        }
        fclose(tempfd);
    }
    if ( saveRawFile )
    {
        string rawFilePath = file + ".raw";
        if ( !replace )
        {
            rawFile = fopen(rawFilePath.c_str(), "rb" );
            if ( rawFile != NULL )
            {
                fclose(rawFile);
                return DAE_ERR_BACKEND_FILE_EXISTS;
            }
            fclose(rawFile);
        }
        rawFile = fopen(rawFilePath.c_str(), "wb");
        if ( rawFile == NULL )
        {
            return DAE_ERR_BACKEND_IO;
        }
        rawRelPath.set(cdom::nativePathToUri(rawFilePath));
        rawRelPath.makeRelativeTo( &name );
    }

    std::string fileName = cdom::uriToNativePath(name.str());
    bool bcompress = fileName.size() >= 4 && fileName[fileName.size()-4] == '.' && ::tolower(fileName[fileName.size()-3]) == 'z' && ::tolower(fileName[fileName.size()-2]) == 'a' && ::tolower(fileName[fileName.size()-1]) == 'e';

    int err=0;
    xmlBufferHandler bufhandler;

    if( bcompress ) {
        // taken from http://xmlsoft.org/examples/testWriter.c
        // Create a new XML buffer, to which the XML document will be written
        bufhandler.buf = xmlBufferCreate();
        if (!bufhandler.buf) {
            ostringstream msg;
            msg << "daeLIBXMLPlugin::write(" << name.str() << ") testXmlwriterMemory: Error creating the xml buffer\n";
            daeErrorHandler::get()->handleError(msg.str().c_str());
            return DAE_ERR_BACKEND_IO;
        }

        // Create a new XmlWriter for memory, with no compression. Remark: there is no compression for this kind of xmlTextWriter
        writer = xmlNewTextWriterMemory(bufhandler.buf, 0);
    }
    else {
        // Open the file we will write to
        writer = xmlNewTextWriterFilename(cdom::fixUriForLibxml(name.str()).c_str(), 0);
    }

    if (!writer) {
        ostringstream msg;
        msg << "daeLIBXMLPlugin::write(" << name.str() << ") Error creating the xml writer\n";
        daeErrorHandler::get()->handleError(msg.str().c_str());
        return DAE_ERR_BACKEND_IO;
    }
    err = xmlTextWriterSetIndentString( writer, (const xmlChar*)"\t" ); // Don't change this to spaces
    if( err < 0 ) {
    }
    err = xmlTextWriterSetIndent( writer, 1 ); // Turns indentation on
    if( err < 0 ) {
    }
    err = xmlTextWriterStartDocument( writer, "1.0", "UTF-8", NULL );
    if( err < 0 ) {
    }

    writeElement( document->getDomRoot() );

    xmlTextWriterEndDocument( writer );
    xmlTextWriterFlush( writer );
    xmlFreeTextWriter( writer );
    writer = NULL; // reset pointer

    if( bcompress ) {
        std::string savefilenameinzip;
        size_t namestart = fileName.find_last_of(s_filesep);
        if( namestart == string::npos ) {
            namestart = 0;
        }
        else {
            namestart+=1;
        }
        if(namestart+4>=fileName.size()) {
            daeErrorHandler::get()->handleError("invalid fileName when removing zae extension");
            return DAE_ERR_BACKEND_IO;
        }
        savefilenameinzip = fileName.substr(namestart,fileName.size()-namestart-4);
        savefilenameinzip += ".dae";

        zipFileHandler zfh;
#ifdef _WIN32
        zlib_filefunc64_def ffunc;
        fill_win32_filefunc64A(&ffunc);
        zfh.zf = zipOpen2_64(fileName.c_str(),APPEND_STATUS_CREATE,NULL,&ffunc);
#else
        zfh.zf = zipOpen64(fileName.c_str(),APPEND_STATUS_CREATE);
#endif
        if (!zfh.zf) {
            ostringstream msg;
            msg << "daeLIBXMLPlugin::write(" << name.str() << ") Error opening zip file for writing\n";
            daeErrorHandler::get()->handleError(msg.str().c_str());
            return DAE_ERR_BACKEND_IO;
        }

        time_t curtime = time(NULL);
        struct tm* timeofday = localtime(&curtime);
        zip_fileinfo zi;
        zi.tmz_date.tm_sec = timeofday->tm_sec;
        zi.tmz_date.tm_min = timeofday->tm_min;
        zi.tmz_date.tm_hour = timeofday->tm_hour;
        zi.tmz_date.tm_mday = timeofday->tm_mday;
        zi.tmz_date.tm_mon = timeofday->tm_mon;
        zi.tmz_date.tm_year = timeofday->tm_year;
        zi.dosDate = 0;
        zi.internal_fa = 0;
        zi.external_fa = 0;

        int zip64 = bufhandler.buf->use >= 0xffffffff;

        char* password=NULL;
        unsigned long crcFile=0;
        int opt_compress_level = 9;
        err = zipOpenNewFileInZip3_64(zfh.zf,savefilenameinzip.c_str(),&zi,NULL,0,NULL,0,"collada file generated by collada-dom",Z_DEFLATED, opt_compress_level,0,-MAX_WBITS, DEF_MEM_LEVEL, Z_DEFAULT_STRATEGY,password,crcFile, zip64);
        if (err != ZIP_OK) {
            ostringstream msg;
            msg << "daeLIBXMLPlugin::write(" << name.str() << ") zipOpenNewFileInZip3_64 error" << err << "\n";
            daeErrorHandler::get()->handleError(msg.str().c_str());
            return DAE_ERR_BACKEND_IO;
        }

        err = zipWriteInFileInZip (zfh.zf,bufhandler.buf->content, bufhandler.buf->use);
        if (err<0) {
            ostringstream msg;
            msg << "daeLIBXMLPlugin::write(" << name.str() << ") zipWriteInFileInZip error for dae file " << err << "\n";
            daeErrorHandler::get()->handleError(msg.str().c_str());
            return DAE_ERR_BACKEND_IO;
        }
        err = zipCloseFileInZip(zfh.zf);
        if (err!=ZIP_OK) {
            ostringstream msg;
            msg << "daeLIBXMLPlugin::write(" << name.str() << ") zipCloseFileInZip error for dae file " << err << "\n";
            daeErrorHandler::get()->handleError(msg.str().c_str());
            return DAE_ERR_BACKEND_IO;
        }

        // add the manifest
        string smanifest = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<dae_root>./";
        smanifest += savefilenameinzip;
        smanifest += "</dae_root>\n";
        err = zipOpenNewFileInZip3_64(zfh.zf,"manifest.xml",&zi,NULL,0,NULL,0,NULL,Z_DEFLATED, opt_compress_level,0,-MAX_WBITS, DEF_MEM_LEVEL, Z_DEFAULT_STRATEGY,password,crcFile, zip64);
        if (err != ZIP_OK) {
            ostringstream msg;
            msg << "daeLIBXMLPlugin::write(" << name.str() << ") zipOpenNewFileInZip3_64 error for manifest.xml file " << err << "\n";
            daeErrorHandler::get()->handleError(msg.str().c_str());
            return DAE_ERR_BACKEND_IO;
        }

        err = zipWriteInFileInZip (zfh.zf,&smanifest[0],smanifest.size());
        if (err != ZIP_OK) {
            ostringstream msg;
            msg << "daeLIBXMLPlugin::write(" << name.str() << ") zipWriteInFileInZip error for manifest.xml file " << err << "\n";
            daeErrorHandler::get()->handleError(msg.str().c_str());
            return DAE_ERR_BACKEND_IO;
        }

        err = zipCloseFileInZip(zfh.zf);
        if (err != ZIP_OK) {
            ostringstream msg;
            msg << "daeLIBXMLPlugin::write(" << name.str() << ") zipCloseFileInZip error for manifest.xml file " << err << "\n";
            daeErrorHandler::get()->handleError(msg.str().c_str());
            return DAE_ERR_BACKEND_IO;
        }
    }

    if ( saveRawFile && rawFile != NULL )
    {
        fclose( rawFile );
    }

    return DAE_OK;
}

void daeLIBXMLPlugin::writeElement( daeElement* element )
{
    daeMetaElement* _meta = element->getMeta();

    //intercept <source> elements for special handling
    if ( saveRawFile )
    {
        if ( strcmp( element->getTypeName(), "source" ) == 0 )
        {
            daeElementRefArray children;
            element->getChildren( children );
            bool validArray = false, teqCommon = false;
            for ( unsigned int i = 0; i < children.getCount(); i++ )
            {
                if ( strcmp( children[i]->getTypeName(), "float_array" ) == 0 ||
                     strcmp( children[i]->getTypeName(), "int_array" ) == 0 )
                {
                    validArray = true;
                }
                else if ( strcmp( children[i]->getTypeName(), "technique_common" ) == 0 )
                {
                    teqCommon = true;
                }
            }
            if ( validArray && teqCommon )
            {
                writeRawSource( element );
                return;
            }
        }
    }

    if (!_meta->getIsTransparent() ) {
        xmlTextWriterStartElement(writer, (xmlChar*)element->getElementName());
        daeMetaAttributeRefArray& attrs = _meta->getMetaAttributes();

        int acnt = (int)attrs.getCount();

        for(int i=0; i<acnt; i++) {
            writeAttribute(attrs[i], element);
        }
    }
    writeValue(element);

    daeElementRefArray children;
    element->getChildren( children );
    for ( size_t x = 0; x < children.getCount(); x++ ) {
        writeElement( children.get(x) );
    }

    /*if (_meta->getContents() != NULL) {
        daeElementRefArray* era = (daeElementRefArray*)_meta->getContents()->getWritableMemory(element);
        int elemCnt = (int)era->getCount();
        for(int i = 0; i < elemCnt; i++) {
            daeElementRef elem = (daeElementRef)era->get(i);
            if (elem != NULL) {
                writeElement( elem );
            }
        }
       }
       else
       {
        daeMetaElementAttributeArray& children = _meta->getMetaElements();
        int cnt = (int)children.getCount();
        for(int i=0;i<cnt;i++) {
            daeMetaElement *type = children[i]->getElementType();
            if ( !type->getIsAbstract() ) {
                for (int c = 0; c < children[i]->getCount(element); c++ ) {
                    writeElement( *(daeElementRef*)children[i]->get(element,c) );
                }
            }
        }
       }*/
    if (!_meta->getIsTransparent() ) {
        xmlTextWriterEndElement(writer);
    }
}

void daeLIBXMLPlugin::writeAttribute( daeMetaAttribute* attr, daeElement* element)
{
    ostringstream buffer;
#ifdef COLLADA_DOM_DAEFLOAT_IS64
    buffer << std::setprecision(std::numeric_limits<PLATFORM_FLOAT64>::digits10+1); // set the default precision to daeFloat digit
#endif
    attr->memoryToString(element, buffer);
    string str = buffer.str();

    // Don't write the attribute if
    //  - The attribute isn't required AND
    //     - The attribute has no default value and the current value is ""
    //     - The attribute has a default value and the current value matches the default
    if (!attr->getIsRequired()) {
        if(!attr->getDefaultValue()  &&  str.empty())
            return;
        if(attr->getDefaultValue()  &&  attr->compareToDefault(element) == 0)
            return;
    }

    xmlTextWriterStartAttribute(writer, (xmlChar*)(daeString)attr->getName());
    xmlChar* utf8 = (xmlChar*)str.c_str();
    if (dae.getCharEncoding() == DAE::Latin1)
        utf8 = latin1ToUtf8(str);
    xmlTextWriterWriteString(writer, utf8);
    if (dae.getCharEncoding() == DAE::Latin1)
        delete[] utf8;

    xmlTextWriterEndAttribute(writer);
}

void daeLIBXMLPlugin::writeValue(daeElement* element) {
    if (daeMetaAttribute* attr = element->getMeta()->getValueAttribute()) {
        ostringstream buffer;
#ifdef COLLADA_DOM_DAEFLOAT_IS64
        buffer << std::setprecision(std::numeric_limits<PLATFORM_FLOAT64>::digits10+1); // set the default precision to daeFloat digit
#endif
        attr->memoryToString(element, buffer);
        string s = buffer.str();
        if (!s.empty()) {
            xmlChar* str = (xmlChar*)s.c_str();
            if (dae.getCharEncoding() == DAE::Latin1)
                str = latin1ToUtf8(s);
            xmlTextWriterWriteString(writer, (xmlChar*)s.c_str());
            if (dae.getCharEncoding() == DAE::Latin1)
                delete[] str;
        }
    }
}

void daeLIBXMLPlugin::writeRawSource( daeElement *src )
{
    daeElementRef newSrc = src->clone();
    daeElementRef array = NULL;
    daeElement *accessor = NULL;
    daeElementRefArray children;
    newSrc->getChildren( children );
    bool isInt = false;
    for ( int i = 0; i < (int)children.getCount(); i++ )
    {
        if ( strcmp( children[i]->getTypeName(), "float_array" ) == 0 )
        {
            array = children[i];
            newSrc->removeChildElement( array );
        }
        else if ( strcmp( children[i]->getTypeName(), "int_array" ) == 0 )
        {
            array = children[i];
            isInt = true;
            newSrc->removeChildElement( array );
        }
        else if ( strcmp( children[i]->getTypeName(), "technique_common" ) == 0 )
        {
            children[i]->getChildren( children );
        }
        else if ( strcmp( children[i]->getTypeName(), "accessor" ) == 0 )
        {
            accessor = children[i];
        }
    }

    daeULong *countPtr = (daeULong*)array->getAttributeValue( "count" );
    daeULong count = countPtr != NULL ? *countPtr : 0;

    daeULong *stridePtr = (daeULong*)accessor->getAttributeValue( "stride" );
    daeULong stride = stridePtr != NULL ? *stridePtr : 1;

    children.clear();
    accessor->getChildren( children );
    if ( children.getCount() > stride ) {
        *stridePtr = children.getCount();
    }

    daeFixedName newURI;
    sprintf( newURI, "%s#%ld", rawRelPath.getOriginalURI(), rawByteCount );
    accessor->setAttribute( "source", newURI );

    daeArray *valArray = (daeArray*)array->getValuePointer();

    //TODO: pay attention to precision for the array.
    if ( isInt )
    {
        for( size_t i = 0; i < count; i++ )
        {
            daeInt tmp = (daeInt)*(daeLong*)(valArray->getRaw(i));
            rawByteCount += (unsigned long)(fwrite( &tmp, sizeof(daeInt), 1, rawFile ) * sizeof(daeInt));
        }
    }
    else
    {
        for( size_t i = 0; i < count; i++ )
        {
            daeFloat tmp = (daeFloat)*(daeDouble*)(valArray->getRaw(i));
            rawByteCount += (unsigned long)(fwrite( &tmp, sizeof(daeFloat), 1, rawFile ) * sizeof(daeFloat));
        }
    }

    writeElement( newSrc );
}

#endif // DOM_INCLUDE_LIBXML
