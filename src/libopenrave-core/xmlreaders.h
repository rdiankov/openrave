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
#ifndef RAVE_XML_READERS
#define RAVE_XML_READERS

#include <libxml/xmlstring.h>

extern int g_XMLErrorCount;
extern map<PluginType, string> g_mapInterfaceNames;

/// Base class for parsing data. Used mostly for XML readers
class BasicStreamParser
{
public:
    virtual ~BasicStreamParser() {}
    virtual void* Format(const char* pdata, int len) = 0;
    virtual int GetCount() const = 0;
    virtual void* GetData() = 0;

    /// releases the pointer (this might be different from GetData()!)
    /// Make sure to deallocate with delete[]
    virtual void* Release() = 0;
                                    
};

/// T is the type, Format is the char to go into sscanf
template <class T, class U>
class TemplateStreamParser : public BasicStreamParser
{
public:
    virtual ~TemplateStreamParser() {}
    virtual void* Format(const U* _pdata, int len)
    {
        T t;
        data.resize(0);
        std::basic_stringstream<U> ss(_pdata);

        while(!ss.eof() ) {
            ss >> t;
            if( !ss )
                break;
            data.push_back(t);
        }

        return data.size() > 0 ? &data[0] : NULL;
    }
    virtual int GetCount() const { return (int)data.size(); }
    virtual void* GetData() { return data.size() > 0 ? &data[0] : NULL; }
    virtual void* Release()
    {
        if( data.size() == 0 ) return NULL;

        T* temp = new T[data.size()];
        memcpy(temp, &data[0], sizeof(T)*data.size());
        data.resize(0);
        return temp;
    }

private:
    std::vector<T> data;
};

// Parses wide strings
class WStringStreamParser : public BasicStreamParser
{
public:
    WStringStreamParser() { p = NULL; }
    virtual ~WStringStreamParser() { delete[] p; }

    virtual void* Format(const char* pdata, int len)
    {
        delete[] p;
        p = new wchar_t[len+1];
        mbstowcs(p, (const char*)pdata, len);
        p[len] = 0;
        return p;
    }

    virtual int GetCount() const { return 1; }
    virtual void* GetData() { return p; }

    virtual void* Release() { wchar_t* temp = p; p = NULL; return temp; }
private:
    wchar_t* p;
};

/// Parses strings
class StringStreamParser : public BasicStreamParser
{
public:
    StringStreamParser() { p = NULL; }
    virtual ~StringStreamParser() { delete[] p; }

    virtual void* Format(const char* pdata, int len)
    {
        delete[] p;
        p = new char[len+1];
        strncpy(p, pdata, len);
        p[len] = 0;
        return p;
    }

    virtual int GetCount() const { return 1; }
    virtual void* GetData() { return p; }

    virtual void* Release() { char* temp = p; p = NULL; return temp; }
private:
    char* p;
};

void RegisterXMLReader(PluginType type, const char* ptype, EnvironmentBase::CreateXMLReaderFn pfn);
void UnregisterXMLReader(PluginType type, const char* ptype);

/////////////////
// XML Readers //
/////////////////

bool RaveParseXMLFile(Environment* penv, BaseXMLReader* preader, const char* filename);
bool RaveParseXMLFile(Environment* penv, BaseXMLReader* preader, const wchar_t* filename);
bool RaveParseXMLData(Environment* penv, BaseXMLReader* preader, const char* pdata, int len);

InterfaceXMLReader* CreateInterfaceReader(Environment* penv, PluginType type, InterfaceBase* pinterface, const string& xmltag, const char** atts);

// mass of objects
struct MASS
{
    MASS() : fTotalMass(0) {}        
    static MASS GetBoxMass(Vector extents, Vector pos, dReal totalmass);
    static MASS GetBoxMassD(Vector extents, Vector pos, dReal density);
    static MASS GetSphericalMass(dReal radius, Vector pos, dReal totalmass);
    static MASS GetSphericalMassD(dReal radius, Vector pos, dReal density);
    static MASS GetCylinderMass(dReal radius, dReal height, Vector pos, dReal totalmass);
    static MASS GetCylinderMassD(dReal radius, dReal height, Vector pos, dReal density);

    /// adds two masses together
    MASS operator+(const MASS& r) const;
    /// adds a mass to the current mass
    MASS& operator+=(const MASS& r);

    /// transform the center of mass and inertia matrix by trans
    MASS& transform(const TransformMatrix& trans);

    TransformMatrix t; 
    dReal fTotalMass;
};

class KinBodyXMLReader;

class LinkXMLReader : public BaseXMLReader
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

    LinkXMLReader(KinBody::Link* plink, KinBody* pparent, const char **atts);
    virtual ~LinkXMLReader() { assert( _pgeomprop == NULL && _plink == NULL ); delete[] _plink; }

    void* Release() { KinBody::Link* temp = _plink; _plink = NULL; return temp; }
    void SetMassType(MassType, float fValue, const Vector& vMassExtents);

    Transform GetOrigTransform() const { return tOrigTrans; }
    KinBodyXMLReader* pKinBodyReader; ///< needed for offsetrom

private:
    
    void startElement(void *ctx ATTRIBUTE_UNUSED, const char *name, const char **atts);

    /// if returns true, XMLReader has finished parsing
    bool endElement(void *ctx ATTRIBUTE_UNUSED, const char *name);
    void characters(void *ctx ATTRIBUTE_UNUSED, const char *ch, int len);

    boost::shared_ptr<BaseXMLReader> _pcurreader;
    BasicStreamParser* _pcurparser; ///< reads the character streams
    MASS _mass;                    ///< current mass of the object
    KinBody::Link* _plink;
    KinBody* _pparent;
    KinBody::Link* _offsetfrom;                    ///< all transformations are relative to the this body
    KinBody::Link::GEOMPROPERTIES* _pgeomprop;

    Transform tOrigTrans;
    // Mass
    MassType _masstype;           ///< if true, mass is craeted so that it mimics the geometry
    bool _bProcessingMass;
    MASS _massCustom;
    float _fMassDensity, _fTotalMass;
    Vector _vMassExtents;           ///< used only if mass is a box
};

// Joint Reader
class JointXMLReader : public BaseXMLReader
{
public:
    JointXMLReader(KinBody* pparent, const char **atts);
    virtual ~JointXMLReader();

    void* Release() { KinBody::Joint* temp = _pjoint; _pjoint = NULL; return temp; }

    bool IsDisabled() const { return bDisabled; }
    bool IsMimic() const { return _bMimicJoint; }
    const string& GetMimicJoint() const { return _strmimicjoint; }

    KinBodyXMLReader* pKinBodyReader; ///< needed for offsetrom

private:
    void startElement(void *ctx ATTRIBUTE_UNUSED, const char *name, const char **atts);

    /// if returns true, XMLReader has finished parsing
    bool endElement(void *ctx ATTRIBUTE_UNUSED, const char *name);
    void characters(void *ctx ATTRIBUTE_UNUSED, const char *ch, int len);

    boost::shared_ptr<BaseXMLReader> _pcurreader;
    BasicStreamParser* _pcurparser; ///< reads the character streams

    float fWeights[3];
    KinBody::Link* _offsetfrom; ///< all transforms are relative to this body
    KinBody* _pparent;
    KinBody::Joint* _pjoint;
    string _strmimicjoint;
    bool bDisabled; // if true, joint is not counted as a controllable degree of freedom
    bool _bMimicJoint;
};

class InterfaceXMLReader : public BaseXMLReader
{
public:
    InterfaceXMLReader(Environment* penv, PluginType type, InterfaceBase* pinterface, const string& xmltag, const char **atts);
    virtual ~InterfaceXMLReader() { delete _pinterface; }

    virtual void* Release() { InterfaceBase* temp = _pinterface; _pinterface = NULL; return temp; }
    InterfaceBase* GetInterface() { return _pinterface; }

protected:
    virtual void startElement(void *ctx ATTRIBUTE_UNUSED, const char *name, const char **atts);
    virtual bool endElement(void *ctx ATTRIBUTE_UNUSED, const char *name);
    virtual void characters(void *ctx ATTRIBUTE_UNUSED, const char *ch, int len);

    Environment* _penv;
    PluginType _type;
    InterfaceBase* _pinterface;
    string _xmltag;
    boost::shared_ptr<BaseXMLReader> _pcustomreader;
    string _interfacename, _readername;
};

// KinBody reader
/// reads kinematic chain specific entries, can instantiate this reader from another reader
class KinBodyXMLReader : public InterfaceXMLReader
{
public:
    KinBodyXMLReader(Environment* penv, PluginType type, KinBody* pchain, const char **atts, int rootoffset, int rootjoffset);
    virtual ~KinBodyXMLReader() { assert(!_pcurreader); }

    const Transform GetOffsetFrom(KinBody::Link* plink);

    string GetModelsDir(const char* pfilename) const;
    void SetXMLFilename(const string& filename) { if( _pchain != NULL && _pchain->strXMLFilename.size() == 0 ) _pchain->strXMLFilename = filename; }
            
protected:
    virtual void startElement(void *ctx ATTRIBUTE_UNUSED, const char *name, const char **atts);

    /// if returns true, XMLReader has finished parsing
    virtual bool endElement(void *ctx ATTRIBUTE_UNUSED, const char *name);
    virtual void characters(void *ctx ATTRIBUTE_UNUSED, const char *ch, int len);

    KinBody* _pchain;
    Transform _trans;
    boost::shared_ptr<BaseXMLReader> _pcurreader;

    // default mass type passed to every LinkXMLReader
    int rootoffset, rootjoffset;                 ///< the initial number of links when KinBody is created (so that global translations and rotations only affect the new links)
    LinkXMLReader::MassType _masstype;             ///< if true, mass is craeted so that it mimics the geometry
    float _fMassValue;               ///< density or total mass
    Vector _vMassExtents;

    vector<Transform> _vTransforms;     ///< original transforms of the bodies for offsetfrom

    string _strModelsDir, _bodyname;
    string prefix;
    wstring wprefix; // add this prefix to all names of links and joints

    RaveVector<float> _diffusecol, _ambientcol;
    float _transparency;
    BasicStreamParser* _pcurparser; ///< reads the character streams
    vector<dReal> _vjointvalues;

    bool _bProcessingMass;
    bool _bOverwriteDiffuse, _bOverwriteAmbient, _bOverwriteTransparency;

    list<pair<KinBody::Joint*,string> > listMimicJoints; ///< mimic joints needed to be resolved
};

class ManipulatorXMLReader : public BaseXMLReader
{
public:
    ManipulatorXMLReader(RobotBase::Manipulator* pmanip, RobotBase* probot, const char **atts);
    virtual ~ManipulatorXMLReader() { delete _pmanip; }

    void* Release() { RobotBase::Manipulator* temp = _pmanip; _pmanip = NULL; return temp; }

protected:
    virtual void startElement(void *ctx ATTRIBUTE_UNUSED, const char *name, const char **atts);

    /// if returns true, XMLReader has finished parsing
    virtual bool endElement(void *ctx ATTRIBUTE_UNUSED, const char *name);
    virtual void characters(void *ctx ATTRIBUTE_UNUSED, const char *ch, int len);

    RobotBase* _probot;
    RobotBase::Manipulator* _pmanip;
    BasicStreamParser* _pcurparser; ///< reads the character streams
    boost::shared_ptr<BaseXMLReader> _pcurreader;
};

/// sensors specifically attached to a robot
class AttachedSensorXMLReader : public BaseXMLReader
{
public:
    AttachedSensorXMLReader(RobotBase::AttachedSensor* psensor, RobotBase* probot, const char **atts);
    virtual ~AttachedSensorXMLReader() { delete _psensor; }

    void* Release() { RobotBase::AttachedSensor* temp = _psensor; _psensor = NULL; return temp; }

protected:
    virtual void startElement(void *ctx ATTRIBUTE_UNUSED, const char *name, const char **atts);

    /// if returns true, XMLReader has finished parsing
    virtual bool endElement(void *ctx ATTRIBUTE_UNUSED, const char *name);
    virtual void characters(void *ctx ATTRIBUTE_UNUSED, const char *ch, int len);

    RobotBase* _probot;
    RobotBase::AttachedSensor* _psensor;
    string args; ///< arguments to pass to sensor when initializing
    BasicStreamParser* _pcurparser; ///< reads the character streams
    boost::shared_ptr<BaseXMLReader> _pcurreader;
};

class RobotXMLReader : public InterfaceXMLReader
{
public:
    RobotXMLReader(Environment* penv, RobotBase* probot, const char **atts, int rootoffset, int rootjoffset, int rootsoffset, int rootmoffset);
    virtual ~RobotXMLReader() { assert(!_pcurreader); }

    void SetXMLFilename(const string& filename) { if( _probot != NULL && _probot->strXMLFilename.size() == 0 ) _probot->strXMLFilename = filename; }

    RobotBase* GetRobot() { return _probot; }

protected:
    virtual void startElement(void *ctx ATTRIBUTE_UNUSED, const char *name, const char **atts);

    /// if returns true, XMLReader has finished parsing
    virtual bool endElement(void *ctx ATTRIBUTE_UNUSED, const char *name);
    virtual void characters(void *ctx ATTRIBUTE_UNUSED, const char *ch, int len);

    RobotBase* _probot;
    BasicStreamParser* _pcurparser; ///< reads the character streams
    boost::shared_ptr<BaseXMLReader> _pcurreader;

    ControllerBase* pNewController; ///< controller to set the robot at
    string strControllerArgs, _strRegReaderId, _robotname;
    string prefix;
    wstring wprefix;

    vector<dReal> _vjointvalues;

    Transform _trans;
    int rootoffset;                 ///< the initial number of links when Robot is created (so that global translations and rotations only affect the new links)
    int rootjoffset; ///< the initial number of joints when Robot is created
    int rootsoffset; ///< the initial number of attached sensors when Robot is created
    int rootmoffset; ///< the initial number of manipulators when Robot is created
    bool bRobotInit;
};

template <PluginType type>
class DummyInterfaceXMLReader : public InterfaceXMLReader
{
public:
    DummyInterfaceXMLReader(Environment* penv, InterfaceBase* pinterface, const string& xmltag, const char** atts) : InterfaceXMLReader(penv,type,pinterface,xmltag,atts) {}
    virtual ~DummyInterfaceXMLReader() {}

    virtual void startElement(void *ctx ATTRIBUTE_UNUSED, const char *name, const char **atts)
    {
        if( !!_pcurreader ) {
            _pcurreader->startElement(ctx, name, atts);
            return;
        }

        InterfaceXMLReader::startElement(ctx,name,atts);
        if( !!_pcustomreader )
            return;
        _pcurreader.reset(new DummyXMLReader(name, g_mapInterfaceNames[_type].c_str()));
    }

    virtual bool endElement(void *ctx ATTRIBUTE_UNUSED, const char *name)
    {
        if( !!_pcurreader ) {
            if( _pcurreader->endElement(ctx, name) )
                _pcurreader.reset();
        }
        else {
            if( InterfaceXMLReader::endElement(ctx,name) )
                return true;
        }
        return false;
    }

    virtual void characters(void *ctx ATTRIBUTE_UNUSED, const char *ch, int len)
    {
        if( !!_pcurreader )
            _pcurreader->characters(ctx,ch,len);
        else
            InterfaceXMLReader::characters(ctx,ch,len);
    }

    boost::shared_ptr<BaseXMLReader> _pcurreader;
};

class ControllerXMLReader : public InterfaceXMLReader
{
public:
    ControllerXMLReader(Environment* penv, InterfaceBase* pinterface, const char** atts);
    virtual ~ControllerXMLReader() {}

    virtual void startElement(void *ctx ATTRIBUTE_UNUSED, const char *name, const char **atts);
    virtual bool endElement(void *ctx ATTRIBUTE_UNUSED, const char *name);
    virtual void characters(void *ctx ATTRIBUTE_UNUSED, const char *ch, int len);

    boost::shared_ptr<BaseXMLReader> _pcurreader;
    wstring _robotname;
    string _args;
};

class ProblemXMLReader : public InterfaceXMLReader
{
public:
    ProblemXMLReader(Environment* penv, InterfaceBase* pinterface, const char** atts);
    virtual ~ProblemXMLReader() {}

    virtual void startElement(void *ctx ATTRIBUTE_UNUSED, const char *name, const char **atts);
    virtual bool endElement(void *ctx ATTRIBUTE_UNUSED, const char *name);
    virtual void characters(void *ctx ATTRIBUTE_UNUSED, const char *ch, int len);

    boost::shared_ptr<BaseXMLReader> _pcurreader;
    string _args;
};

class EnvironmentXMLReader : public BaseXMLReader
{
public:
    EnvironmentXMLReader(Environment* penv, const char **atts);
    virtual ~EnvironmentXMLReader();

    void* Release() { Environment* temp = _penv; _penv = NULL; return temp; }

protected:
    virtual void startElement(void *ctx ATTRIBUTE_UNUSED, const char *name, const char **atts);

    /// if returns true, XMLReader has finished parsing
    virtual bool endElement(void *ctx ATTRIBUTE_UNUSED, const char *name);
    virtual void characters(void *ctx ATTRIBUTE_UNUSED, const char *ch, int len);

    Environment* _penv;
    BasicStreamParser* _pcurparser; ///< reads the character streams
    boost::shared_ptr<BaseXMLReader> _pcurreader;

    Vector vBkgndColor;
    Transform tCamera; ///< default camera transformationn

    bool bTransSpecified;
    bool _bInEnvironment;
};

#endif
