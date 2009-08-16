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
/*! --------------------------------------------------------------------
  \file   rave.h
  \brief  Defines the public headers that every plugin must include
    in order to use openrave properly.
 -------------------------------------------------------------------- */

#ifndef RAVE_RAVE_H
#define RAVE_RAVE_H

#include <cstdio>
#include <stdarg.h>
#include <cstring>
#include <cstdlib>
#include <stdint.h>

#ifdef _MSC_VER
// needed to get typeof working
//#include <boost/typeof/std/string.hpp>
//#include <boost/typeof/std/vector.hpp>
//#include <boost/typeof/std/list.hpp>
//#include <boost/typeof/std/map.hpp>
//#include <boost/typeof/std/set.hpp>
//#include <boost/typeof/std/string.hpp>

#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __FUNCDNAME__
#endif

#else
#endif

#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>

#include <iomanip>
#include <fstream>
#include <sstream>

namespace OpenRAVE {

#include <rave/defines.h>
#include <rave/classhashes.h>

// terminal attributes
//#define RESET           0
//#define BRIGHT          1
//#define DIM             2
//#define UNDERLINE       3
//#define BLINK           4
//#define REVERSE         7
//#define HIDDEN          8
// terminal colors
//#define BLACK           0
//#define RED             1
//#define GREEN           2
//#define YELLOW          3
//#define BLUE            4
//#define MAGENTA         5
//#define CYAN            6
//#define WHITE           7

/// Change the text color (on either stdout or stderr) with an attr:fg:bg (thanks to Radu Rusu for the code)
inline std::string ChangeTextColor (int attribute, int fg, int bg)
{
    char command[13];
    sprintf (command, "%c[%d;%d;%dm", 0x1B, attribute, fg + 30, bg + 40);
    //fwprintf (stream, L"%s", command);
    return command;
}

inline std::wstring ChangeTextColorW (int attribute, int fg, int bg)
{
    wchar_t command[13];
    swprintf (command, 13, L"%c[%d;%d;%dm", 0x1B, attribute, fg + 30, bg + 40);
    return command;
}

/// Change the text color (on either stdout or stderr) with an attr:fg (thanks to Radu Rusu for the code)
inline std::string ChangeTextColor (int attribute, int fg)
{
    char command[13];
    sprintf (command, "%c[%d;%dm", 0x1B, attribute, fg + 30);
    return command;
    //fwprintf (stream, L"%s", command);
}

inline std::wstring ChangeTextColorW (int attribute, int fg)
{
    wchar_t command[13];
    swprintf (command, 13, L"%c[%d;%dm", 0x1B, attribute, fg + 30);
    return command;
}

/// Reset the text color (on either stdout or stderr) to its original state (thanks to Radu Rusu for the code)
inline std::string ResetTextColor ()
{
    char command[13];
    sprintf (command, "%c[0;m", 0x1B);
    return command;
    //fwprintf (stream, L"%s", command);
}

inline std::wstring ResetTextColorW ()
{
    wchar_t command[13];
    swprintf (command, 13, L"%c[0;m", 0x1B);
    return command;
}

inline std::wstring RavePrintTransformString(const wchar_t* fmt)
{
    std::vector<int> positions;
    std::wstring str = fmt;
    wchar_t* p = wcsstr(&str[0], L"%s");
    while(p != NULL ) {
        positions.push_back((int)(p-&str[0])+1);
        p = wcsstr(p+2, L"%s");
    }

    p = wcsstr(&str[0], L"%S");
    while(p != NULL ) {
        p[1] = 's';
        p = wcsstr(p+2, L"%S");
    }

    p = wcsstr(&str[0], L"%ls");
    while(p != NULL ) {
        p[1] = 's';
        p[2] = ' ';
        p = wcsstr(p+2, L"%ls");
    }

    for(int i = 0; i < (int)positions.size(); ++i)
        str[positions[i]] = 'S';
    return str;
}

enum DebugLevel {
    Level_Fatal=0,
    Level_Error=1,
    Level_Warn=2,
    Level_Info=3,
    Level_Debug=4,
    Level_Verbose=5
};

#define OPENRAVECOLOR_FATALLEVEL 5 // magenta
#define OPENRAVECOLOR_ERRORLEVEL 1 // red
#define OPENRAVECOLOR_WARNLEVEL 3 // yellow
#define OPENRAVECOLOR_INFOLEVEL 0 // black
#define OPENRAVECOLOR_DEBUGLEVEL 2 // green
#define OPENRAVECOLOR_VERBOSELEVEL 4 // blue

#ifdef _WIN32

#define DefineRavePrintfW(LEVEL) \
    inline int RavePrintfW##LEVEL(const wchar_t *fmt, ...) \
    { \
        /*ChangeTextColor (stdout, 0, OPENRAVECOLOR##LEVEL);*/ \
        va_list list; \
	    va_start(list,fmt); \
        int r = vwprintf(OpenRAVE::RavePrintTransformString(fmt).c_str(), list); \
        va_end(list); \
        /*ResetTextColor (stdout);*/ \
        return r; \
    }

#define DefineRavePrintfA(LEVEL) \
    inline int RavePrintfA##LEVEL(const char *fmt, ...) \
    { \
        /*ChangeTextColor (stdout, 0, OPENRAVECOLOR##LEVEL);*/ \
        va_list list; \
        va_start(list,fmt); \
        int r = vprintf(fmt, list); \
        va_end(list); \
        /*ResetTextColor(stdout);*/ \
        return r; \
    }

#else

#define DefineRavePrintfW(LEVEL) \
    inline int RavePrintfW##LEVEL(const wchar_t *wfmt, ...) \
    { \
        va_list list; \
        va_start(list,wfmt); \
        /* Allocate memory on the stack to avoid heap fragmentation */ \
        size_t allocsize = wcstombs(NULL, wfmt, 0)+32; \
        char* fmt = (char*)alloca(allocsize); \
        strcpy(fmt, ChangeTextColor(0, OPENRAVECOLOR##LEVEL).c_str()); \
        snprintf(fmt+strlen(fmt),allocsize-16,"%S",wfmt); \
        strcat(fmt, ResetTextColor().c_str()); \
        int r = vprintf(fmt, list);        \
        va_end(list); \
        return r; \
    }

// In linux, only wprintf will succeed, due to the fwide() call in main, so
// for programmers who want to use regular format strings without
// the L in front, we will take their regular string and widen it
// for them.
#define DefineRavePrintfA(LEVEL) \
    inline int RavePrintfA##LEVEL(const char *fmt, ...) \
    { \
        va_list list; \
	    va_start(list,fmt); \
        int r = vprintf((ChangeTextColor(0, OPENRAVECOLOR##LEVEL) + std::string(fmt) + ResetTextColor()).c_str(), list); \
        va_end(list); \
        return r; \
    }

#endif

DefineRavePrintfW(_FATALLEVEL)
DefineRavePrintfW(_ERRORLEVEL)
DefineRavePrintfW(_WARNLEVEL)
DefineRavePrintfW(_INFOLEVEL)
DefineRavePrintfW(_DEBUGLEVEL)
DefineRavePrintfW(_VERBOSELEVEL)

DefineRavePrintfA(_FATALLEVEL)
DefineRavePrintfA(_ERRORLEVEL)
DefineRavePrintfA(_WARNLEVEL)
DefineRavePrintfA(_INFOLEVEL)
DefineRavePrintfA(_DEBUGLEVEL)
DefineRavePrintfA(_VERBOSELEVEL)

/// extracts only the filename
inline const char* RaveGetSourceFilename(const char* pfilename)
{
    if( pfilename == NULL )
        return "";

    const char* p0 = strrchr(pfilename,'/');
    const char* p1 = strrchr(pfilename,'\\');
    const char* p = p0 > p1 ? p0 : p1;
    if( p == NULL )
        return pfilename;

    return p+1;
}

#define RAVEPRINTHEADER(LEVEL) OpenRAVE::RavePrintfW##LEVEL(L"[%s:%d] ", OpenRAVE::RaveGetSourceFilename(__FILE__), __LINE__)

// different logging levels. The higher the suffix number, the less important the information is.
// 0 log level logs all the time. OpenRAVE starts up with a log level of 0.
#define RAVELOG_LEVEL(LEVEL,level) OpenRAVE::RaveGetDebugLevel()>=(level)&&(RAVEPRINTHEADER(LEVEL)>0)&&OpenRAVE::RavePrintfW##LEVEL
#define RAVELOG_LEVELA(LEVEL,level) OpenRAVE::RaveGetDebugLevel()>=(level)&&(RAVEPRINTHEADER(LEVEL)>0)&&OpenRAVE::RavePrintfA##LEVEL

// define log4cxx equivalents (eventually OpenRAVE will move to log4cxx logging)
#define RAVELOG_FATAL RAVELOG_LEVEL(_FATALLEVEL,OpenRAVE::Level_Fatal)
#define RAVELOG_FATALA RAVELOG_LEVELA(_FATALLEVEL,OpenRAVE::Level_Fatal)
#define RAVELOG_ERROR RAVELOG_LEVEL(_ERRORLEVEL,OpenRAVE::Level_Error)
#define RAVELOG_ERRORA RAVELOG_LEVELA(_ERRORLEVEL,OpenRAVE::Level_Error)
#define RAVELOG_WARN RAVELOG_LEVEL(_WARNLEVEL,OpenRAVE::Level_Warn)
#define RAVELOG_WARNA RAVELOG_LEVELA(_WARNLEVEL,OpenRAVE::Level_Warn)
#define RAVELOG_INFO RAVELOG_LEVEL(_INFOLEVEL,OpenRAVE::Level_Info)
#define RAVELOG_INFOA RAVELOG_LEVELA(_INFOLEVEL,OpenRAVE::Level_Info)
#define RAVELOG_DEBUG RAVELOG_LEVEL(_DEBUGLEVEL,OpenRAVE::Level_Debug)
#define RAVELOG_DEBUGA RAVELOG_LEVELA(_DEBUGLEVEL,OpenRAVE::Level_Debug)
#define RAVELOG_VERBOSE RAVELOG_LEVEL(_VERBOSELEVEL,OpenRAVE::Level_Verbose)
#define RAVELOG_VERBOSEA RAVELOG_LEVELA(_VERBOSELEVEL,OpenRAVE::Level_Verbose)

// deprecated
#define RAVELOG RAVELOG_DEBUG
#define RAVELOGA RAVELOG_DEBUGA
#define RAVEPRINT RAVELOG_INFO
#define RAVEPRINTA RAVELOG_INFOA

#define IS_DEBUGLEVEL(level) (OpenRAVE::RaveGetDebugLevel()>=(level))

struct PLUGININFO
{
    std::vector<std::wstring> robots;
    std::vector<std::wstring> planners;
    std::vector<std::wstring> sensorsystems;
    std::vector<std::wstring> controllers;
    std::vector<std::wstring> problems;
    std::vector<std::wstring> iksolvers;
    std::vector<std::wstring> physicsengines;
    std::vector<std::wstring> sensors;
    std::vector<std::wstring> collisioncheckers;
    std::vector<std::wstring> trajectories;
    std::vector<std::wstring> viewers;
    std::vector<std::wstring> servers;
};

enum PluginType
{
    PT_Planner=1, ///< PlannerBase interface
    PT_Robot=2, ///< RobotBase interface
    PT_SensorSystem=3, ///< SensorSystemBase interface
    PT_Controller=4, ///< ControllerBase interface
    PT_ProblemInstance=5, ///< ProblemInstance interface
    PT_InverseKinematicsSolver=6, ///< IkSolverBase interface
    PT_KinBody=7, ///< arbitrary KinBody
    PT_PhysicsEngine=8, ///< physics simulation engine
    PT_Sensor=9, ///< sensor like camera, laser range finder, tactile
    PT_CollisionChecker=10, ///< collision checker
    PT_Trajectory=11, ///< holds a trajectory of configuration space points (also performs various smoothing and filtering steps)
    PT_Viewer=12,///< a viewer for the OpenRAVE state. Each environment can attach one viewer to it
    PT_Server=13,///< a server used for communicating OpenRAVE state across the network
};

class COLLISIONREPORT;
class IkSolverBase;
class Trajectory;
class ControllerBase;
class PlannerBase;
class RobotBase;
class ProblemInstance;
class EnvironmentBase;
class KinBody;
class SensorSystemBase;
class PhysicsEngineBase;
class SensorBase;
class CollisionCheckerBase;
class RaveViewerBase;
class RaverServerBase;

class EnvironmentBase;

///< Cloning Options for interfaces and environments
enum CloningOptions {
    Clone_Bodies = 1, ///< clone all the bodies/robots of the environment
    Clone_Viewer = 2, ///< clone the viewer type, although figures won't be copied, new viewer does try to match views
    Clone_Simulation = 4, ///< clone the physics engine and simulation state (ie, timesteps, gravity)
    Clone_RealControllers = 8 ///< if specified, will clone the real controllers of all the robots, otherwise each robot gets ideal controller
};

/// base class for readable interfaces
class XMLReadable
{
public:
    XMLReadable(const std::string& xmlid) : __xmlid(xmlid) {}
	virtual ~XMLReadable() {}
    virtual const std::string& GetXMLId() const { return __xmlid; }
private:
    std::string __xmlid;
};

/// base class for all interfaces that OpenRAVE provides
class InterfaceBase
{
public:
    InterfaceBase(PluginType type, EnvironmentBase* penv) : __type(type), __penv(penv), __pUserData(NULL) {}
	virtual ~InterfaceBase() {
        for(std::map<std::string, XMLReadable* >::iterator it = __mapReadableInterfaces.begin(); it != __mapReadableInterfaces.end(); ++it) {
            delete it->second;
        }
        __mapReadableInterfaces.clear();
    }

    inline PluginType GetInterfaceType() const { return __type; }

    /// set internally by RaveDatabase
	/// \return the unique identifier that describes this class type, case is ignored
    /// should be the same id used to create the object
    inline const char* GetXMLId() const { return __strxmlid.c_str(); }

    /// set internally by RaveDatabase
    /// \return the pluginname this interface was loaded from
    inline const char* GetPluginName() const { return __strpluginname.c_str(); }

    /// \return the environment that this interface is attached to
    inline EnvironmentBase* GetEnv() const { return __penv; }

    inline const std::map<std::string, XMLReadable* >& GetReadableInterfaces() const { return __mapReadableInterfaces; }

    virtual void SetUserData(void* pdata) { __pUserData = pdata; }
    virtual void* GetUserData() const { return __pUserData; }
    
    /// clone the contents of an interface to the current interface
    /// \param preference the interface whose information to clone
    /// \param cloningoptions mask of CloningOptions
    virtual bool Clone(const InterfaceBase* preference, int cloningoptions) { return true; }

protected:
    virtual const char* GetHash() const = 0;

private:
    std::string __strpluginname, __strxmlid;
    PluginType __type;
    EnvironmentBase* __penv;
    void* __pUserData;                       ///< data set by the user
    std::map<std::string, XMLReadable* > __mapReadableInterfaces; ///< pointers to extra interfaces that are included with this object

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
    friend class RaveDatabase;
    friend class InterfaceXMLReader;
#else
    friend class ::RaveDatabase;
    friend class ::InterfaceXMLReader;
#endif
#endif
};

inline DebugLevel RaveGetDebugLevel(void);

/// base class for all xml readers. XMLReaders are used to process data from
/// xml files. Custom readers can be registered through EnvironmentBase
class BaseXMLReader
{
public:
    virtual ~BaseXMLReader() {}

    virtual void* Release() = 0;

    /// Gets called in the beginning of each "<type>" expression. In this case, name is "type"
    /// \param ctx context of the XML parser
    /// \param name of the tag
    /// \param atts NULL terminated arary of string of attributes. For example in <type myatt="openrave">
    ///             atts[0] = "myatt", atts[1] = "openrave", atts[2] = NULL
    virtual void startElement(void *ctx, const char *name, const char **atts) = 0;

    /// Gets called at the end of each "</type>" expression. In this case, name is "type"
    /// \param ctx context of the XML parser
    /// \param name of the tag
    /// \return true if XMLReader has finished parsing, otherwise false
    virtual bool endElement(void *ctx, const char *name) = 0;

    /// gets called for all data in between tags.
    /// \param ctx context of the XML parser
    /// \param ch pointer to the data, only the first len characters are valid (not NULL terminated!)
    /// \param len length of valid data
    virtual void characters(void *ctx, const char *ch, int len) = 0;

    std::string _filename; /// XML filename
};

class DummyXMLReader : public BaseXMLReader
{
public:
    DummyXMLReader(const char* pfieldname, const char* pparentname);
    
    virtual void* Release();
    
    virtual void startElement(void *ctx, const char *name, const char **atts);
    
    /// if returns true, XMLReader has finished parsing
    virtual bool endElement(void *ctx, const char *name);
    virtual void characters(void *ctx, const char *ch, int len) {}
    
private:
    std::string _fieldname, _parentname; /// XML filename
    BaseXMLReader* _pcurreader;
};

class OneTagReader : public BaseXMLReader
{
public:
    OneTagReader(std::string tag, BaseXMLReader* preader);

    virtual void* Release();

    virtual void startElement(void *ctx, const char *name, const char **atts);
    virtual bool endElement(void *ctx, const char *name);
    virtual void characters(void *ctx, const char *ch, int len);

private:
    BaseXMLReader* _preader;
    std::string _tag;
    int _numtags;
};

} // end namespace OpenRAVE


#include <rave/math.h>
#include <rave/kinbody.h>
#include <rave/trajectory.h>
#include <rave/problems.h>
#include <rave/collisionreport.h>
#include <rave/collisionchecker.h>
#include <rave/sensor.h>
#include <rave/robot.h>
#include <rave/planner.h>
#include <rave/controller.h>
#include <rave/physicsengine.h>
#include <rave/sensorsystem.h>
#include <rave/iksolver.h>
#include <rave/viewer.h>
#include <rave/server.h>

namespace OpenRAVE {

/// Environment class
/// Holds everything necessary to load the environment, simulate a problem, and gather statistics.
/// It should be free of rendering and GUI work.
class EnvironmentBase
{
public:
    typedef BaseXMLReader* (*CreateXMLReaderFn)(InterfaceBase* pinterface, const char **atts);

    // gets the state of all bodies that should be published to the GUI
    struct BODYSTATE
    {
        BODYSTATE() : pbody(NULL) {}
        KinBody* pbody;
        std::vector<RaveTransform<dReal> > vectrans;
        std::vector<dReal> jointvalues;
        void* pguidata;
        std::wstring strname; ///< name of the body
        int networkid; ///< unique network id
    };

    class EnvLock
    {
    public:
        virtual ~EnvLock() {}
    };

    virtual ~EnvironmentBase() {}

    /// Resets all objects of the scene (preserves all problems, planners, and server state)
    /// do not call inside a SimulationStep call
    virtual void Reset()=0;

    //@{ plugin info
    /// get all the loaded plugins and the interfaces they support
    /// \param plugins A list of plugins. Each entry has the plugin name and the interfaces it supports
    virtual void GetPluginInfo(std::list< std::pair<std::string, PLUGININFO> >& plugins)=0;

    /// get a list of all the loaded interfaces
    virtual void GetLoadedInterfaces(PLUGININFO& info) = 0;

    /// load a plugin and its interfaces
    virtual bool LoadPlugin(const char* pname) = 0;
    //@}

    virtual InterfaceBase* CreateInterface(PluginType type,const char* pinterfacename)=0;
    virtual RobotBase* CreateRobot(const wchar_t* pname)=0;
    virtual RobotBase* CreateRobot(const char* pname)=0;
    virtual PlannerBase* CreatePlanner(const wchar_t* pname)=0;
    virtual PlannerBase* CreatePlanner(const char* pname)=0;
    virtual SensorSystemBase* CreateSensorSystem(const wchar_t* pname)=0;
    virtual SensorSystemBase* CreateSensorSystem(const char* pname)=0;
    virtual ControllerBase* CreateController(const wchar_t* pname)=0;
    virtual ControllerBase* CreateController(const char* pname)=0;
    virtual ProblemInstance* CreateProblem(const wchar_t* pname)=0;
    virtual ProblemInstance* CreateProblem(const char* pname)=0;
    virtual IkSolverBase* CreateIkSolver(const wchar_t* pname)=0;
    virtual IkSolverBase* CreateIkSolver(const char* pname)=0;
    virtual PhysicsEngineBase* CreatePhysicsEngine(const wchar_t* pname)=0;
    virtual PhysicsEngineBase* CreatePhysicsEngine(const char* pname)=0;
    virtual SensorBase* CreateSensor(const wchar_t* pname)=0;
    virtual SensorBase* CreateSensor(const char* pname)=0;
    virtual CollisionCheckerBase* CreateCollisionChecker(const wchar_t* pname)=0;
    virtual CollisionCheckerBase* CreateCollisionChecker(const char* pname)=0;
    virtual RaveViewerBase* CreateViewer(const wchar_t* pname)=0;
    virtual RaveViewerBase* CreateViewer(const char* pname)=0;
    virtual RaveServerBase* CreateServer(const wchar_t* pname)=0;
    virtual RaveServerBase* CreateServer(const char* pname)=0;

    /// Returns a clone of the current environment. Clones do not share any memory or resource between each other
    /// or their parent making them ideal for performing separte planning experiments while keeping
    /// the parent environment unchanged.
    /// By default a clone only copies the collision checkers and physics engine.
    /// When bodies are cloned, the unique ids are preserved across environments (each body can be referenced with its id in both environments). The attached and grabbed bodies of each body/robot are also copied to the new environment.
    /// \param options A set of CloningOptions describing what is actually cloned.
    virtual EnvironmentBase* CloneSelf(int options) = 0;

    /// Collision specific functions
    //@{
    virtual bool SetCollisionChecker(CollisionCheckerBase* pchecker)=0;
    virtual CollisionCheckerBase* GetCollisionChecker() const =0;

    virtual bool SetCollisionOptions(int options)=0;
    virtual int GetCollisionOptions() const = 0;

    virtual bool CheckCollision(const KinBody* pbody1, COLLISIONREPORT* pReport = NULL)=0;
    virtual bool CheckCollision(const KinBody* pbody1, const KinBody* pbody2, COLLISIONREPORT* pReport = NULL)=0;

    virtual bool CheckCollision(const KinBody::Link* plink, COLLISIONREPORT* pReport = NULL)=0;
    virtual bool CheckCollision(const KinBody::Link* plink1, const KinBody::Link* plink2, COLLISIONREPORT* pReport = NULL)=0;
    
    virtual bool CheckCollision(const KinBody::Link* plink, const KinBody* pbody, COLLISIONREPORT* pReport = NULL)=0;
    
    virtual bool CheckCollision(const KinBody::Link* plink, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, COLLISIONREPORT* pReport = NULL)=0;
    virtual bool CheckCollision(const KinBody* pbody, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, COLLISIONREPORT* pReport = NULL)=0;
    
    /// Check collision with a link and a ray with a specified length.
    /// \param ray holds the origin and direction. The length of the ray is the length of the direction.
    /// \param plink the link to collide with
    virtual bool CheckCollision(const RAY& ray, const KinBody::Link* plink, COLLISIONREPORT* pReport = NULL) = 0;

    /// Check collision with a body and a ray with a specified length.
    /// \param ray holds the origin and direction. The length of the ray is the length of the direction.
    /// \param pbody the kinbody to look for collisions
    virtual bool CheckCollision(const RAY& ray, const KinBody* pbody, COLLISIONREPORT* pReport = NULL) = 0;

    /// Check collision with the environment and a ray with a specified length.
    /// \param ray holds the origin and direction. The length of the ray is the length of the direction.
    virtual bool CheckCollision(const RAY& ray, COLLISIONREPORT* pReport = NULL) = 0;
	
    //tolerance check
    virtual bool CheckCollision(const KinBody::Link* plink, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, dReal tolerance)= 0;
    virtual bool CheckCollision(const KinBody* pbody, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, dReal tolerance)= 0;
    //@}

    ///@{ file I/O

    /// Loads a scene, need to LockPhysics if calling outside simulation thread
    virtual bool   Load(const char *filename) = 0;
    /// Loads a scene, need to LockPhysics if calling outside simulation thread
    virtual bool   Load(const wchar_t *filename) = 0;
    /// Saves a scene depending on the filename extension. Default is in COLLADA format
    virtual bool   Save(const char* filename) = 0;

    /// Initializes a robot from an XML file.
    /// \param robot If a null pointer is passed, a new robot will be created, otherwise an existing robot will be filled
    virtual RobotBase* ReadRobotXML(RobotBase* robot, const char* filename, const char** atts) = 0;

    /// Initializes a kinematic body from an XML file.
    /// \param robot If a null pointer is passed, a new body will be created, otherwise an existing robot will be filled
    virtual KinBody* ReadKinBodyXML(KinBody* robot, const char* filename, const char** atts) = 0;
    ///@}

    /// Objects
    virtual bool AddKinBody(KinBody* pbody) = 0;
    virtual bool AddRobot(RobotBase* robot) = 0;

    /// Removes  KinBody from the environment. If bDestroy is true, also
    /// deallocates the KinBody memory (physics also needs to be locked!). 
    virtual bool RemoveKinBody(KinBody* pbody, bool bDestroy = true) = 0;

    /// \return first KinBody (including robots) that matches with pname
    virtual KinBody* GetKinBody(const wchar_t* pname)=0;

    /// Get the corresponding body from its unique network id
    virtual KinBody* GetBodyFromNetworkId(int id) = 0;

    /// \return an empty KinBody instance, deallocate with delete, physics needs to be locked
    virtual KinBody* CreateKinBody() = 0;

    /// \return an empty trajectory instance initialized to nDOF degrees of freedom. Deallocate with delete.
    virtual Trajectory* CreateTrajectory(int nDOF) = 0;

    /// Load a new problem, need to LockPhysics if calling outside simulation thread
    virtual int LoadProblem(ProblemInstance* prob, const char* cmdargs) = 0;
    /// Load a new problem, need to LockPhysics if calling outside simulation thread
    virtual bool RemoveProblem(ProblemInstance* prob) = 0;
    /// Get a list of loaded problems
    virtual const std::list<ProblemInstance*>& GetProblems() const = 0;

    /// Random number generation
    //@{
    /// generate a random integer, 32bit precision
    virtual unsigned int RandomInt() = 0;
    /// generate n random integers, 32bit precision
    virtual void RandomInt(unsigned int n, std::vector<int>& v) = 0;
    /// generate a random float in [0,1], 32bit precision
    virtual float RandomFloat() = 0;
    /// generate n random floats in [0,1], 32bit precision
    virtual void RandomFloat(unsigned int n, std::vector<float>& v) = 0;
    /// generate a random double in [0,1], 53bit precision
    virtual double RandomDouble() = 0;
    /// generate n random doubles in [0,1], 53bit precision
    virtual void RandomDouble(unsigned int n, std::vector<double>& v) = 0;
    //@}

    //@{ Physics/Simulation methods

    /// set the physics engine, disabled by default
    /// \param the engine to set, if NULL, environment sets an dummy physics engine
    virtual bool SetPhysicsEngine(PhysicsEngineBase* pengine) = 0;
    virtual PhysicsEngineBase* GetPhysicsEngine() const = 0;

    /// Makes one simulation step (will be rounded up to the nearest microsecond)
    virtual void StepSimulation(dReal timeStep) = 0;

    /// Start the internal physics engine loop, calls SimulateStep for all modules
    virtual void StartSimulation(dReal fDeltaTime) = 0;

    /// Stops the internal physics loop, stops calling SimulateStep for all modules
    virtual void StopSimulation() = 0;
    
    /// \return simulation time since the start of the environment (in microseconds)
    virtual uint64_t GetSimulationTime() = 0;
    //@}

    /// Lock/unlock a mutex controlling the physics simulation thread.
    /// If this mutex is locked, then the simulation thread will block
    /// without calling any phsyics functions. Any SimulationStep functions across
    /// interfaces automatically have physics locked.
    /// \param bLock if true will acquire the lock
    /// \param timeout if nonzero, will wait that many seconds until giving up. if zero, will wait forever
    /// \return returns true if the operation successfully completed
    virtual bool LockPhysics(bool bLock, float timeout=0) = 0;

    virtual bool AttachViewer(RaveViewerBase* pnewviewer) = 0;
    virtual RaveViewerBase* GetViewer() const = 0;

    /// All plotting calls are thread safe
    //@{ 3D drawing methods

    /// plots 3D points.
    /// \return handle to plotted points
    /// \param ppoints array of points
    /// \param numPoints number of points to plot
    /// \param stride stride in bytes to next point, ie: nextpoint = (float*)((char*)ppoints+stride)
    /// \param fPointSize size of a point in pixels
    /// \param color the rgb color of the point. The last component of the color is used for alpha blending
    /// \param drawstyle if 0 will draw pixels. if 1, will draw 3D spheres
    virtual void* plot3(const float* ppoints, int numPoints, int stride, float fPointSize = 1.0f, const RaveVector<float>& color = RaveVector<float>(1,0.5,0.5,1), int drawstyle = 0) = 0;

    /// plots 3D points. Arguments same as plot3 with one color, except has an individual color for every point
    /// \param colors An array of rgb colors of size numPoints where each channel is in [0,1].
    ///               colors+3 points to the second color.
    /// \param drawstyle if 0 will draw pixels. if 1, will draw 3D spherse
    /// \return handle to plotted points
    virtual void* plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle = 0) = 0;
    
    /// draws a series of connected lines
    /// \param color the rgb color of the point. The last component of the color is used for alpha blending
    virtual void* drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth = 1.0f, const RaveVector<float>& color = RaveVector<float>(1,0.5,0.5,1)) = 0;
    virtual void* drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors) = 0;

    /// draws a list of individual lines, each specified by a succeeding pair of points
    /// \param color the rgb color of the point. The last component of the color is used for alpha blending.
    virtual void* drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth = 1.0f, const RaveVector<float>& color = RaveVector<float>(1,0.5,0.5,1)) = 0;
    virtual void* drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors) = 0;

    /// draws an arrow p1 is start, p2 is finish
    /// \param color the rgb color of the point. The last component of the color is used for alpha blending.
    virtual void* drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth = 0.002f, const RaveVector<float>& color = RaveVector<float>(1,0.5,0.5,1)) = 0;
    
    /// draws a box
    /// extents are half the width, height, and depth of the box
    /// \return handle to box
    virtual void* drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents) = 0;

    /// draws a triangle mesh, each vertices of each triangle should be counter-clockwise.
    /// \param ppoints - array of 3D points
    /// \param stride stride in bytes to next point, ie: nextpoint = (float*)((char*)ppoints+stride)
    /// \param pIndices If not NULL, zero-based indices into the points for every triangle. pIndices
    /// should be of size numTriangles. If pIndices is NULL, ppoints is assumed to contain numTriangles*3
    /// points and triangles will be rendered in list order.
    /// \param color The color of the triangle. The last component of the color is used for alpha blending
    virtual void* drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color = RaveVector<float>(1,0.5,0.5,1)) = 0;
    //virtual void* drawtrimesh(const float* ppoints, const int* pIndices, int numTriangles, const float* colors);

    /// closes previous graphs
    virtual void closegraph(void* handle) = 0;
    //@}

    //@{ Other GUI interaction methods

    /// Set the camera transformation
    virtual void SetCamera(const RaveTransform<float>& trans) = 0;

    /// Set the camera transformation
    /// \param pos position of origin of camera
    /// \param quat orientation of camera in quaterions
    virtual void SetCamera(const RaveVector<float>& pos, const RaveVector<float>& quat) = 0;

    /// Set the camera transformation while looking at a particular point in space
    /// \param lookat the point space to look at, the camera will rotation and zoom around this point
    /// \param campos the position of the camera in space
    /// \param the up vector from the camera
    virtual void SetCameraLookAt(const RaveVector<float>& lookat, const RaveVector<float>& campos, const RaveVector<float>& camup) = 0;
    virtual RaveTransform<float> GetCameraTransform() = 0;

    virtual bool GetFractionOccluded(KinBody* pbody, int width, int height, float nearPlane, float farPlane, const RaveTransform<float>& extrinsic, const float* pKK, double& fracOccluded) = 0;

    /// Renders a 24bit RGB image of dimensions width and height from the current scene. The camera
    /// is meant to show the underlying OpenRAVE world as a robot would see it, so all graphs
    /// rendered with the plotX and drawX functions are hidden.
    /// \param pMemory the memory where the image will be stored at, has to store 3*width*height
    /// \param width width of the image
    /// \param height height of the image
    /// \param t the rotation and translation of the camera. Note that z is treated as the front of the camera!
    ///        So all points in front of the camera have a positive dot product with its direction.
    /// \param pKK 4 values such that the intrinsic matrix can be reconstructed [pKK[0] 0 pKK[2]; 0 pKK[1] pKK[3]; 0 0 1];
    virtual bool GetCameraImage(void* pMemory, int width, int height, const RaveTransform<float>& t, const float* pKK) = 0;

    /// Renders a 24bit RGB image of dimensions width and height from the current scene and saves it to a file.
    /// The camera is meant to show the underlying OpenRAVE world as a robot would see it, so all graphs
    /// rendered with the plotX and drawX functions are hidden.
    /// \param width width of the image
    /// \param height height of the image
    /// \param t the rotation and translation of the camera. Note that z is treated as the front of the camera!
    ///        So all points in front of the camera have a positive dot product with its direction.
    /// \param pKK 4 values such that the intrinsic matrix can be reconstructed [pKK[0] 0 pKK[2]; 0 pKK[1] pKK[3]; 0 0 1];
    virtual bool WriteCameraImage(int width, int height, const RaveTransform<float>& t, const float* pKK, const char* fileName, const char* extension) = 0;
    //@}

    /// return all robots
    virtual const std::vector<RobotBase*>& GetRobots() const = 0;

    /// fill an array with all bodies loaded in the environment (including roobts)
    virtual void GetBodies(std::vector<KinBody*>& bodies) const = 0;
    
    /// retrieve published bodies, note that the pbody pointer might become invalid
    /// as soon as GetPublishedBodies returns
    virtual void GetPublishedBodies(std::vector<BODYSTATE>& vbodies) = 0;
    virtual void SetPublishBodiesAnytime(bool bAnytime) = 0;
    virtual bool GetPublishBodiesAnytime() const = 0;

    /// returns the global kinbody array
    virtual const std::vector<KinBody*>& GetBodies() const = 0;

    /// Returns a set of bodies and locks the environment from creating and destroying new bodies
    /// (ie, creation of and destruction functions will block until lock is released).
    /// In order to release the lock, delete the EnvLock pointer returned.
    /// \param bodies Fills with all the body pointers in the environment
    /// \return returns a pointer to a Lock
    virtual EnvLock* GetLockedBodies(std::vector<KinBody*>& bodies) const = 0;

    /// Returns a set of robots and locks the environment from creating and destroying new bodies
    /// (ie, creation of and destruction functions will block until lock is released).
    /// In order to release the lock, delete the EnvLock pointer returned.
    /// \param bodies Fills with all the body pointers in the environment
    /// \return returns a pointer to a Lock
    virtual EnvLock* GetLockedRobots(std::vector<RobotBase*>& robots) const = 0;

    /// returns the openrave home directory where settings, cache, and other files are stored.
    /// On Linux/Unix systems, this is usually $HOME/.openrave, on Windows this is $HOMEPATH/.openrave
    virtual const char* GetHomeDirectory() const = 0;

    /// XML processing functions.
    //@{

    /// registers a custom xml reader for a particular interface. Once registered, anytime
    /// the tag specified in xmltag is seen in the interface, the the custom reader will be created.
    virtual void RegisterXMLReader(PluginType type, const char* xmltag, CreateXMLReaderFn pfn) = 0;

    /// unregisters the xml reader
    virtual void UnregisterXMLReader(PluginType type, const char* xmltag) = 0;
    
    /// Parses a file for XML data
    virtual bool ParseXMLFile(BaseXMLReader* preader, const char* filename) = 0;

    /// Parses a data file for XML data
    /// \param pdata The data of the buffer
    /// \param len the number of bytes valid in pdata
    virtual bool ParseXMLData(BaseXMLReader* preader, const char* pdata, int len) = 0;
    //@}

    enum TriangulateOptions
    {
        TO_Obstacles = 1,   ///< everything but robots
        TO_Robots = 2,
        TO_Everything = 3,  ///< all KinBodies
        TO_Body = 4,        ///< only triangulate kinbody
        TO_AllExceptBody = 5 ///< triangulate everything but kinbody
    };
    
    /// triangulation of the body including its current transformation. trimesh will be appended the new data.
    virtual bool Triangulate(KinBody::Link::TRIMESH& trimesh, const KinBody* pbody) = 0;

    /// general triangulation of the whole scene. trimesh will be appended the new data.
    /// \param opts - Controlls what to triangulate
    virtual bool TriangulateScene(KinBody::Link::TRIMESH& trimesh, TriangulateOptions opts, const wchar_t* pName) = 0;

    //@{ debug/global commands
    
    /// sets the debug level
    /// \param level 0 for no debug, 1 - to print all debug messeges. Default
    ///             value for release builds is 0, for debug builds it is 1
    /// declaring variables with stdcall can be a little complex
    virtual void SetDebugLevel(DebugLevel level) = 0;
    virtual DebugLevel GetDebugLevel() const = 0;
    //@}

    //@{ environment server
    virtual bool AttachServer(RaveServerBase* pserver) = 0;
    virtual RaveServerBase* GetServer() const = 0;
    //@}
};

inline void RaveSetDebugLevel(DebugLevel level) {
    extern DebugLevel g_nDebugLevel;
    g_nDebugLevel = level;
}

inline DebugLevel RaveGetDebugLevel(void) {
    extern DebugLevel g_nDebugLevel;
    return g_nDebugLevel;
}

//@}

} // end namespace OpenRAVE

// register for typeof (MSVC only)
#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::ProblemInstance)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::CmdProblemInstance)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::SensorSystemBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::ControllerBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::PlannerBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::PlannerBase::PlannerParameters)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::IkSolverBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::SensorBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::SensorBase::SensorData)
BOOST_TYPEOF_REGISTER_TEMPLATE(OpenRAVE::RaveVector, 1)
BOOST_TYPEOF_REGISTER_TEMPLATE(OpenRAVE::RaveTransform, 1)
BOOST_TYPEOF_REGISTER_TEMPLATE(OpenRAVE::RaveTransformMatrix, 1)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::Joint)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::Link)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::Link::GEOMPROPERTIES)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::Link::TRIMESH)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::KinBodyStateSaver)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RobotBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RobotBase::Manipulator)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RobotBase::AttachedSensor)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RobotBase::GRABBED)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RobotBase::RobotStateSaver)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::Trajectory)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::Trajectory::TPOINT)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::Trajectory::TSEGMENT)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::PLUGININFO)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::XMLReadable)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::InterfaceBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::BaseXMLReader)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::EnvironmentBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::EnvironmentBase::BODYSTATE)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RAY)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::AABB)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::OBB)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::TRIANGLE)
#endif


#endif
