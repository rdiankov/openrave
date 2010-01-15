// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
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

#ifndef RAVE_DISABLE_ASSERT_HANDLER
#define BOOST_ENABLE_ASSERT_HANDLER
#endif

#include <cstdio>
#include <stdarg.h>
#include <cstring>
#include <cstdlib>
#include <stdint.h>

#ifdef _MSC_VER

#pragma warning(disable:4251) // needs to have dll-interface to be used by clients of class
#pragma warning(disable:4190) // C-linkage specified, but returns UDT 'boost::shared_ptr<T>' which is incompatible with C

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
#include <exception>

#include <iomanip>
#include <fstream>
#include <sstream>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/enable_shared_from_this.hpp> 
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/static_assert.hpp>
#include <boost/format.hpp>
#include <boost/array.hpp>

#if defined(_MSC_VER) && (defined(RAVE_USEDLL) || defined(RAVE_CORE_USEDLL))
#ifdef RAVE_LIBBUILD
#define RAVE_API __declspec(dllexport)
#else
#define RAVE_API __declspec(dllimport)
#endif
#else
#define RAVE_API 
#endif

// export symbol prefix for plugin functions
#ifdef _MSC_VER
#define RAVE_PLUGIN_API extern "C" __declspec(dllexport)
#else
#define RAVE_PLUGIN_API extern "C"
#endif

namespace OpenRAVE {

#include <rave/defines.h>
#include <rave/classhashes.h>

enum OpenRAVEErrorCode {
    ORE_Failed=0,
    ORE_InvalidArguments=1,
    ORE_EnvironmentNotLocked=2,
    ORE_CommandNotSupported=3,
    ORE_Assert=4,
};

enum SerializationOptions
{
    SO_Kinematics = 0x01, ///< kinematics information
    SO_Dynamics = 0x02, ///< dynamics information
    SO_BodyState = 0x04, ///< state of the body
    SO_NamesAndFiles = 0x08, ///< resource files and names
    SO_RobotManipulators = 0x10, ///< serialize robot manipulators
    SO_RobotSensors = 0x20, ///< serialize robot sensors
};

struct openrave_exception : std::exception
{
    openrave_exception() : std::exception(), _s("unknown exception"), _error(ORE_Failed) {}
    openrave_exception(const std::string& s, OpenRAVEErrorCode error=ORE_Failed) : std::exception() { _s = "OpenRAVE: " + s; _error = error; }
    virtual ~openrave_exception() throw() {}
    char const* what() const throw() { return _s.c_str(); }
    OpenRAVEErrorCode GetCode() const { return _error; }
private:
    std::string _s;
    OpenRAVEErrorCode _error;
};

class CaseInsentiveCompare
{
 public:
    bool operator()(const std::string & s1, const std::string& s2) const
    {
        std::string::const_iterator it1=s1.begin();
        std::string::const_iterator it2=s2.begin();
        
        //has the end of at least one of the strings been reached?
        while ( (it1!=s1.end()) && (it2!=s2.end()) )  { 
            if(::toupper(*it1) != ::toupper(*it2)) //letters differ?
                // return -1 to indicate 'smaller than', 1 otherwise
                return ::toupper(*it1) < ::toupper(*it2);
            //proceed to the next character in each string
            ++it1;
            ++it2;
        }
        std::size_t size1=s1.size(), size2=s2.size();// cache lengths
        //return -1,0 or 1 according to strings' lengths
        if (size1==size2) 
            return 0;
        return size1<size2;
    }
};

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
    sprintf (command, "%c[0;38;48m", 0x1B);
    return command;
    //fwprintf (stream, L"%s", command);
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
    inline int RavePrintfA##LEVEL(const std::string& s) \
    { \
        printf ("%s", s.c_str()); \
        return s.size(); \
    } \
    \
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
        strcpy(fmt, ChangeTextColor(0, OPENRAVECOLOR##LEVEL,8).c_str()); \
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
    inline int RavePrintfA##LEVEL(const std::string& s) \
    { \
        printf ("%c[0;%d;%dm%s%c[0;38;48m", 0x1B, OPENRAVECOLOR##LEVEL + 30,8+40,s.c_str(),0x1B); \
        return s.size(); \
    } \
    \
    inline int RavePrintfA##LEVEL(const char *fmt, ...) \
    { \
        va_list list; \
	    va_start(list,fmt); \
        int r = vprintf((ChangeTextColor(0, OPENRAVECOLOR##LEVEL,8) + std::string(fmt) + ResetTextColor()).c_str(), list); \
        va_end(list); \
        return r; \
    } \

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

#define RAVEPRINTHEADER(LEVEL) OpenRAVE::RavePrintfA##LEVEL("[%s:%d] ", OpenRAVE::RaveGetSourceFilename(__FILE__), __LINE__)

// different logging levels. The higher the suffix number, the less important the information is.
// 0 log level logs all the time. OpenRAVE starts up with a log level of 0.
#define RAVELOG_LEVELW(LEVEL,level) OpenRAVE::RaveGetDebugLevel()>=(level)&&(RAVEPRINTHEADER(LEVEL)>0)&&OpenRAVE::RavePrintfW##LEVEL
#define RAVELOG_LEVELA(LEVEL,level) OpenRAVE::RaveGetDebugLevel()>=(level)&&(RAVEPRINTHEADER(LEVEL)>0)&&OpenRAVE::RavePrintfA##LEVEL

// define log4cxx equivalents (eventually OpenRAVE will move to log4cxx logging)
#define RAVELOG_FATALW RAVELOG_LEVELW(_FATALLEVEL,OpenRAVE::Level_Fatal)
#define RAVELOG_FATALA RAVELOG_LEVELA(_FATALLEVEL,OpenRAVE::Level_Fatal)
#define RAVELOG_FATAL RAVELOG_FATALA
#define RAVELOG_ERRORW RAVELOG_LEVELW(_ERRORLEVEL,OpenRAVE::Level_Error)
#define RAVELOG_ERRORA RAVELOG_LEVELA(_ERRORLEVEL,OpenRAVE::Level_Error)
#define RAVELOG_ERROR RAVELOG_ERRORA
#define RAVELOG_WARNW RAVELOG_LEVELW(_WARNLEVEL,OpenRAVE::Level_Warn)
#define RAVELOG_WARNA RAVELOG_LEVELA(_WARNLEVEL,OpenRAVE::Level_Warn)
#define RAVELOG_WARN RAVELOG_WARNA
#define RAVELOG_INFOW RAVELOG_LEVELW(_INFOLEVEL,OpenRAVE::Level_Info)
#define RAVELOG_INFOA RAVELOG_LEVELA(_INFOLEVEL,OpenRAVE::Level_Info)
#define RAVELOG_INFO RAVELOG_INFOA
#define RAVELOG_DEBUGW RAVELOG_LEVELW(_DEBUGLEVEL,OpenRAVE::Level_Debug)
#define RAVELOG_DEBUGA RAVELOG_LEVELA(_DEBUGLEVEL,OpenRAVE::Level_Debug)
#define RAVELOG_DEBUG RAVELOG_DEBUGA
#define RAVELOG_VERBOSEW RAVELOG_LEVELW(_VERBOSELEVEL,OpenRAVE::Level_Verbose)
#define RAVELOG_VERBOSEA RAVELOG_LEVELA(_VERBOSELEVEL,OpenRAVE::Level_Verbose)
#define RAVELOG_VERBOSE RAVELOG_VERBOSEA

#define IS_DEBUGLEVEL(level) (OpenRAVE::RaveGetDebugLevel()>=(level))

RAVE_API void RaveSetDebugLevel(DebugLevel level);

inline DebugLevel RaveGetDebugLevel(void) {
#ifdef RAVE_LIBBUILD
    extern DebugLevel g_nDebugLevel;
#else
    RAVE_API extern DebugLevel g_nDebugLevel;
#endif
    return g_nDebugLevel;
}

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
};

struct PLUGININFO
{
    std::map<PluginType, std::vector<std::string> > interfacenames;
};

class COLLISIONREPORT;
class InterfaceBase;
class IkSolverBase;
class TrajectoryBase;
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

typedef boost::shared_ptr<COLLISIONREPORT> CollisionReportPtr;
typedef boost::shared_ptr<COLLISIONREPORT const> CollisionReportConstPtr;
typedef boost::shared_ptr<InterfaceBase> InterfaceBasePtr;
typedef boost::shared_ptr<InterfaceBase const> InterfaceBaseConstPtr;
typedef boost::weak_ptr<InterfaceBase> InterfaceBaseWeakPtr;
typedef boost::shared_ptr<KinBody> KinBodyPtr;
typedef boost::shared_ptr<KinBody const> KinBodyConstPtr;
typedef boost::weak_ptr<KinBody> KinBodyWeakPtr;
typedef boost::shared_ptr<RobotBase> RobotBasePtr;
typedef boost::shared_ptr<RobotBase const> RobotBaseConstPtr;
typedef boost::weak_ptr<RobotBase> RobotBaseWeakPtr;
typedef boost::shared_ptr<CollisionCheckerBase> CollisionCheckerBasePtr;
typedef boost::shared_ptr<CollisionCheckerBase const> CollisionCheckerBaseConstPtr;
typedef boost::weak_ptr<CollisionCheckerBase> CollisionCheckerBaseWeakPtr;
typedef boost::shared_ptr<ControllerBase> ControllerBasePtr;
typedef boost::shared_ptr<ControllerBase const> ControllerBaseConstPtr;
typedef boost::weak_ptr<ControllerBase> ControllerBaseWeakPtr;
typedef boost::shared_ptr<IkSolverBase> IkSolverBasePtr;
typedef boost::shared_ptr<IkSolverBase const> IkSolverBaseConstPtr;
typedef boost::weak_ptr<IkSolverBase> IkSolverBaseWeakPtr;
typedef boost::shared_ptr<PhysicsEngineBase> PhysicsEngineBasePtr;
typedef boost::shared_ptr<PhysicsEngineBase const> PhysicsEngineBaseConstPtr;
typedef boost::weak_ptr<PhysicsEngineBase> PhysicsEngineBaseWeakPtr;
typedef boost::shared_ptr<PlannerBase> PlannerBasePtr;
typedef boost::shared_ptr<PlannerBase const> PlannerBaseConstPtr;
typedef boost::weak_ptr<PlannerBase> PlannerBaseWeakPtr;
typedef boost::shared_ptr<ProblemInstance> ProblemInstancePtr;
typedef boost::shared_ptr<ProblemInstance const> ProblemInstanceConstPtr;
typedef boost::weak_ptr<ProblemInstance> ProblemInstanceWeakPtr;
typedef boost::shared_ptr<SensorBase> SensorBasePtr;
typedef boost::shared_ptr<SensorBase const> SensorBaseConstPtr;
typedef boost::weak_ptr<SensorBase> SensorBaseWeakPtr;
typedef boost::shared_ptr<SensorSystemBase> SensorSystemBasePtr;
typedef boost::shared_ptr<SensorSystemBase const> SensorSystemBaseConstPtr;
typedef boost::weak_ptr<SensorSystemBase> SensorSystemBaseWeakPtr;
typedef boost::shared_ptr<TrajectoryBase> TrajectoryBasePtr;
typedef boost::shared_ptr<TrajectoryBase const> TrajectoryBaseConstPtr;
typedef boost::weak_ptr<TrajectoryBase> TrajectoryBaseWeakPtr;
typedef boost::shared_ptr<RaveViewerBase> RaveViewerBasePtr;
typedef boost::shared_ptr<RaveViewerBase const> RaveViewerBaseConstPtr;
typedef boost::weak_ptr<RaveViewerBase> RaveViewerBaseWeakPtr;
typedef boost::shared_ptr<EnvironmentBase> EnvironmentBasePtr;
typedef boost::shared_ptr<EnvironmentBase const> EnvironmentBaseConstPtr;
typedef boost::weak_ptr<EnvironmentBase> EnvironmentBaseWeakPtr;

///< Cloning Options for interfaces and environments
enum CloningOptions {
    Clone_Bodies = 1, ///< clone all the bodies/robots of the environment
    Clone_Viewer = 2, ///< clone the viewer type, although figures won't be copied, new viewer does try to match views
    Clone_Simulation = 4, ///< clone the physics engine and simulation state (ie, timesteps, gravity)
    Clone_RealControllers = 8 ///< if specified, will clone the real controllers of all the robots, otherwise each robot gets ideal controller
};

/// base class for readable interfaces
class RAVE_API XMLReadable
{
public:
    XMLReadable(const std::string& xmlid) : __xmlid(xmlid) {}
	virtual ~XMLReadable() {}
    virtual const std::string& GetXMLId() const { return __xmlid; }
private:
    std::string __xmlid;
};

typedef boost::shared_ptr<XMLReadable> XMLReadablePtr;
typedef boost::shared_ptr<XMLReadable const> XMLReadableConstPtr;

/// base class for all xml readers. XMLReaders are used to process data from
/// xml files. Custom readers can be registered through EnvironmentBase
class RAVE_API BaseXMLReader : public boost::enable_shared_from_this<BaseXMLReader>
{
public:
    BaseXMLReader() : __bRecordXMLData(true) {}
    virtual ~BaseXMLReader() {}

    /// a readable interface that stores the information processsed for the current tag
    /// This pointer is used to the InterfaceBase class registered readers
    virtual XMLReadablePtr GetReadable() { return XMLReadablePtr(); }

    /// Gets called in the beginning of each "<type>" expression. In this case, name is "type"
    /// \param name of the tag, will be always lower case
    /// \param atts string of attributes where the first std::string is the attribute name and second is the value
    virtual void startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts);

    /// Gets called at the end of each "</type>" expression. In this case, name is "type"
    /// \param name of the tag, will be always lower case
    /// \return true if XMLReader has finished parsing, otherwise false
    virtual bool endElement(const std::string& name);

    /// gets called for all data in between tags.
    /// \param ch a string to the data
    virtual void characters(const std::string& ch);
    
    /// returns the XML formatted data that this reader has parsed so far
    virtual std::string GetXMLData() const { return __sxml.str(); }

    std::string _filename; /// XML filename

protected:
    bool __bRecordXMLData;
    std::stringstream __sxml; ///< used to store the xml data
};
typedef boost::shared_ptr<BaseXMLReader> BaseXMLReaderPtr;
typedef boost::shared_ptr<BaseXMLReader const> BaseXMLReaderConstPtr;

class RAVE_API DummyXMLReader : public BaseXMLReader
{
public:
    DummyXMLReader(const std::string& pfieldname, const std::string& pparentname);
    
    virtual void startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts);
    
    /// if returns true, XMLReader has finished parsing
    virtual bool endElement(const std::string& name);
    
private:
    std::string _fieldname, _parentname; /// XML filename
    boost::shared_ptr<BaseXMLReader> _pcurreader;
};

class RAVE_API OneTagReader : public BaseXMLReader
{
public:
    OneTagReader(const std::string& tag, BaseXMLReaderPtr preader);

    virtual void startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts);
    virtual bool endElement(const std::string& name);
    virtual void characters(const std::string& ch);

private:
    boost::shared_ptr<BaseXMLReader> _preader;
    std::string _tag;
    int _numtags;
};

} // end namespace OpenRAVE


#include <rave/math.h>
#include <rave/interface.h>
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

namespace OpenRAVE {

typedef boost::recursive_try_mutex EnvironmentMutex;

/// Environment class
/// Holds everything necessary to load the environment, simulate a problem, and gather statistics.
/// It should be free of rendering and GUI work.
class EnvironmentBase : public boost::enable_shared_from_this<EnvironmentBase>
{
public:
    EnvironmentBase() {}
    virtual ~EnvironmentBase() {}

    /// releases all environment resources. Should be called whenever the environment stops being used
    /// (removing all environment pointer might not be enough to destroy the environment resources)
    virtual void Destroy()=0;

    /// Resets all objects of the scene (preserves all problems, planners)
    /// do not call inside a SimulationStep call
    virtual void Reset()=0;

    //@{ plugin info
    /// get all the loaded plugins and the interfaces they support
    /// \param plugins A list of plugins. Each entry has the plugin name and the interfaces it supports
    virtual void GetPluginInfo(std::list< std::pair<std::string, PLUGININFO> >& plugins)=0;

    /// get a list of all the loaded interfaces
    virtual void GetLoadedInterfaces(PLUGININFO& info) = 0;

    /// load a plugin and its interfaces
    virtual bool LoadPlugin(const std::string& pname) = 0;

    /// returns true if interface can be loaded from a plugin, otherwise false
    virtual bool HasInterface(PluginType type, const std::string& interfacename) = 0;
    //@}

    virtual InterfaceBasePtr CreateInterface(PluginType type,const std::string& interfacename)=0;
    virtual RobotBasePtr CreateRobot(const std::string& name="")=0;
    virtual PlannerBasePtr CreatePlanner(const std::string& name)=0;
    virtual SensorSystemBasePtr CreateSensorSystem(const std::string& name)=0;
    virtual ControllerBasePtr CreateController(const std::string& name)=0;
    virtual ProblemInstancePtr CreateProblem(const std::string& name)=0;
    virtual IkSolverBasePtr CreateIkSolver(const std::string& name)=0;
    virtual PhysicsEngineBasePtr CreatePhysicsEngine(const std::string& name)=0;
    virtual SensorBasePtr CreateSensor(const std::string& name)=0;
    virtual CollisionCheckerBasePtr CreateCollisionChecker(const std::string& name)=0;
    virtual RaveViewerBasePtr CreateViewer(const std::string& name)=0;

    /// \return an empty KinBody instance, deallocate with delete, physics needs to be locked
    virtual KinBodyPtr CreateKinBody(const std::string& name="") = 0;

    /// \return an empty trajectory instance initialized to nDOF degrees of freedom. Deallocate with delete.
    virtual TrajectoryBasePtr CreateTrajectory(int nDOF) = 0;

    /// environment will own the interface until Destroy is called
    virtual void OwnInterface(InterfaceBasePtr pinterface) = 0;

    /// environment owner if interface is removed
    virtual void DisownInterface(InterfaceBasePtr pinterface) = 0;

    /// Returns a clone of the current environment. Clones do not share any memory or resource between each other
    /// or their parent making them ideal for performing separte planning experiments while keeping
    /// the parent environment unchanged.
    /// By default a clone only copies the collision checkers and physics engine.
    /// When bodies are cloned, the unique ids are preserved across environments (each body can be referenced with its id in both environments). The attached and grabbed bodies of each body/robot are also copied to the new environment.
    /// \param options A set of CloningOptions describing what is actually cloned.
    virtual EnvironmentBasePtr CloneSelf(int options) = 0;

    /// Collision specific functions. Each function takes an optional pointer to a CollisionReport structure and returns true if collision occurs.
    //@{
    virtual bool SetCollisionChecker(CollisionCheckerBasePtr pchecker)=0;
    virtual CollisionCheckerBasePtr GetCollisionChecker() const =0;

    virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr report = CollisionReportPtr())=0;
    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report = CollisionReportPtr())=0;
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr())=0;
    virtual bool CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr report = CollisionReportPtr())=0;
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr())=0;
    
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report = CollisionReportPtr())=0;
    virtual bool CheckCollision(KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report = CollisionReportPtr())=0;

    /// Check collision with a link and a ray with a specified length.
    /// \param ray holds the origin and direction. The length of the ray is the length of the direction.
    /// \param plink the link to collide with
    /// \param report an optional collision report to be filled with data about the collision. If a body was hit, CollisionReport::plink1 contains the hit link pointer.
    virtual bool CheckCollision(const RAY& ray, KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr()) = 0;

    /// Check collision with a link and a ray with a specified length.
    /// \param ray holds the origin and direction. The length of the ray is the length of the direction.
    /// \param plink the link to collide with
    /// \param report an optional collision report to be filled with data about the collision. If a body was hit, CollisionReport::plink1 contains the hit link pointer.
    virtual bool CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) = 0;

    /// Check collision with a body and a ray with a specified length.
    /// \param ray holds the origin and direction. The length of the ray is the length of the direction.
    /// \param pbody the kinbody to look for collisions
    /// \param report an optional collision report to be filled with data about the collision. If a body was hit, CollisionReport::plink1 contains the hit link pointer.
    virtual bool CheckCollision(const RAY& ray, CollisionReportPtr report = CollisionReportPtr()) = 0;

    virtual bool CheckSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) = 0;
    //@}

    /// Register a callback to be called whenever a collision is detected between between bodies during a CheckCollision call or physics simulation.
    /// action = callback(CollisionReport,bool IsCalledFromPhysicsEngine)
    /// The callback should return an action specifying how the collision should be handled.
    /// \return a handle to the registration, once the handle loses scope, the callback is unregistered
    typedef boost::function<CollisionAction(CollisionReportPtr,bool)> CollisionCallbackFn;
    virtual boost::shared_ptr<void> RegisterCollisionCallback(const CollisionCallbackFn& callback) = 0;
    virtual bool HasRegisteredCollisionCallbacks() const = 0;
    virtual void GetRegisteredCollisionCallbacks(std::list<CollisionCallbackFn>&) const = 0;

    //@{ Physics/Simulation methods

    /// set the physics engine, disabled by default
    /// \param the engine to set, if NULL, environment sets an dummy physics engine
    virtual bool SetPhysicsEngine(PhysicsEngineBasePtr pengine) = 0;
    virtual PhysicsEngineBasePtr GetPhysicsEngine() const = 0;

    /// Makes one simulation step
    virtual void StepSimulation(dReal timeStep) = 0;

    /// Start the internal physics engine loop, calls SimulateStep for all modules
    /// \param fDeltaTime the delta step to take in simulation
    /// \param bRealTime if false will call SimulateStep as fast as possible, otherwise will time the simulate step calls so that simulation progresses with real system time.
    virtual void StartSimulation(dReal fDeltaTime, bool bRealTime=true) = 0;

    /// Stops the internal physics loop, stops calling SimulateStep for all modules
    virtual void StopSimulation() = 0;

    /// \return true if inner simulation loop is executing
    virtual bool IsSimulationRunning() const = 0;
    
    /// \return simulation time since the start of the environment (in microseconds)
    virtual uint64_t GetSimulationTime() = 0;
    //@}

    ///@{ file I/O

    /// Loads a scene, need to Lock if calling outside simulation thread
    virtual bool Load(const std::string& filename) = 0;
    /// Saves a scene depending on the filename extension. Default is in COLLADA format
    virtual bool Save(const std::string& filename) = 0;

    /// Initializes a robot from an XML file. The robot should not be added the environment when calling this function.
    /// \param robot If a null pointer is passed, a new robot will be created, otherwise an existing robot will be filled
    /// \param filename the name of the file to open
    /// \param atts the XML attributes/value pairs
    virtual RobotBasePtr ReadRobotXMLFile(RobotBasePtr robot, const std::string& filename, const std::list<std::pair<std::string,std::string> >& atts) = 0;

    /// Initialize a robot from an XML formatted string
    /// The robot should not be added the environment when calling this function.
    /// \param robot If a null pointer is passed, a new robot will be created, otherwise an existing robot will be filled
    /// \param atts the XML attributes/value pairs
    virtual RobotBasePtr ReadRobotXMLData(RobotBasePtr robot, const std::string& data, const std::list<std::pair<std::string,std::string> >& atts) = 0;

    /// Initializes a kinematic body from an XML file. The body should not be added to the environment when calling this function.
    /// \param filename the name of the file to open
    /// \param body If a null pointer is passed, a new body will be created, otherwise an existing robot will be filled
    /// \param atts the XML attributes/value pairs
    virtual KinBodyPtr ReadKinBodyXMLFile(KinBodyPtr body, const std::string& filename, const std::list<std::pair<std::string,std::string> >& atts) = 0;

    /// Initializes a kinematic body from an XML formatted string.
    // The body should not be added to the environment when calling this function.
    /// \param body If a null pointer is passed, a new body will be created, otherwise an existing robot will be filled
    /// \param atts the XML attributes/value pairs
    virtual KinBodyPtr ReadKinBodyXMLData(KinBodyPtr body, const std::string& data, const std::list<std::pair<std::string,std::string> >& atts) = 0;

    /// Initializes an interface from an XML file.
    /// \param pinterface If a null pointer is passed, a new interface will be created, otherwise an existing interface will be filled
    /// \param filename the name of the file to open
    /// \param atts the XML attributes/value pairs
    virtual InterfaceBasePtr ReadInterfaceXMLFile(InterfaceBasePtr pinterface, PluginType type, const std::string& filename, const std::list<std::pair<std::string,std::string> >& atts) = 0;

    /// Initializes an interface from an XML formatted string.
    /// \param pinterface If a null pointer is passed, a new interface will be created, otherwise an existing interface will be filled
    /// \param data string containing XML data
    /// \param atts the XML attributes/value pairs
    virtual InterfaceBasePtr ReadInterfaceXMLData(InterfaceBasePtr pinterface, PluginType type, const std::string& data, const std::list<std::pair<std::string,std::string> >& atts) = 0;
    ///@}

    /// Objects
    virtual bool AddKinBody(KinBodyPtr pbody) = 0;
    virtual bool AddRobot(RobotBasePtr robot) = 0;

    /// Removes  KinBody from the environment. If bDestroy is true, also
    /// deallocates the KinBody memory (physics also needs to be locked!). 
    virtual bool RemoveKinBody(KinBodyPtr pbody) = 0;

    /// \return first KinBody (including robots) that matches with name
    virtual KinBodyPtr GetKinBody(const std::string& name)=0;

    /// \return first Robot that matches with name
    virtual RobotBasePtr GetRobot(const std::string& name)=0;

    /// Get the corresponding body from its unique network id
    virtual KinBodyPtr GetBodyFromNetworkId(int id) = 0;

    /// Load a new problem, need to Lock if calling outside simulation thread
    virtual int LoadProblem(ProblemInstancePtr prob, const std::string& cmdargs) = 0;
    /// Load a new problem, need to Lock if calling outside simulation thread
    virtual bool RemoveProblem(ProblemInstancePtr prob) = 0;

    /// Returns a list of loaded problems with a lock. As long as the lock is held, the problems
    /// are guaranteed to stay loaded in the environment.
    /// \return returns a pointer to a Lock. Destroying the shared_ptr will release the lock
    virtual boost::shared_ptr<void> GetLoadedProblems(std::list<ProblemInstancePtr>& listProblems) const = 0;

    /// Lock/unlock the environment mutex. Accessing environment body information and adding/removing bodies
    /// or changing any type of scene property should have the environment lock acquired. Once the environment
    /// is locked, the user is guaranteed that nnothing will change in the environment.
    /// \return the mutex used to control the lock.
    virtual EnvironmentMutex& GetMutex() const = 0;

    virtual bool AttachViewer(RaveViewerBasePtr pnewviewer) = 0;
    virtual RaveViewerBasePtr GetViewer() const = 0;

    /// All plotting calls are thread safe
    //@{ 3D drawing methods

    typedef boost::shared_ptr<void> GraphHandlePtr;

    /// plots 3D points.
    /// \param ppoints array of points
    /// \param numPoints number of points to plot
    /// \param stride stride in bytes to next point, ie: nextpoint = (float*)((char*)ppoints+stride)
    /// \param fPointSize size of a point in pixels
    /// \param color the rgb color of the point. The last component of the color is used for alpha blending
    /// \param drawstyle if 0 will draw pixels. if 1, will draw 3D spheres
    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color = RaveVector<float>(1,0.5,0.5,1), int drawstyle = 0) = 0;

    /// plots 3D points. Arguments same as plot3 with one color, except has an individual color for every point
    /// \param colors An array of rgb colors of size numPoints where each channel is in [0,1].
    ///               colors+(bhasalpha?4:3) points to the second color.
    /// \param drawstyle if 0 will draw pixels. if 1, will draw 3D spherse
    /// \param bhasalpha if true, then each color consists of 4 values with the last value being the alpha of the point (1 means opaque). If false, then colors is 3 values.
    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle = 0, bool bhasalpha = false) = 0;
    
    /// draws a series of connected lines
    /// \param color the rgb color of the point. The last component of the color is used for alpha blending
    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color = RaveVector<float>(1,0.5,0.5,1)) = 0;

    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors) = 0;

    /// draws a list of individual lines, each specified by a succeeding pair of points
    /// \param color the rgb color of the point. The last component of the color is used for alpha blending.
    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color = RaveVector<float>(1,0.5,0.5,1)) = 0;

    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors) = 0;

    /// draws an arrow p1 is start, p2 is finish
    /// \param color the rgb color of the point. The last component of the color is used for alpha blending.
    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual GraphHandlePtr drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color = RaveVector<float>(1,0.5,0.5,1)) = 0;
    
    /// draws a box
    /// extents are half the width, height, and depth of the box
    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual GraphHandlePtr drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents) = 0;

    /// draws a triangle mesh, each vertices of each triangle should be counter-clockwise.
    /// \param ppoints - array of 3D points
    /// \param stride stride in bytes to next point, ie: nextpoint = (float*)((char*)ppoints+stride)
    /// \param pIndices If not NULL, zero-based indices into the points for every triangle. pIndices
    /// should be of size numTriangles. If pIndices is NULL, ppoints is assumed to contain numTriangles*3
    /// points and triangles will be rendered in list order.
    /// \param color The color of the triangle. The last component of the color is used for alpha blending
    /// \return handle to plotted points, graph is removed when handle is destroyed (goes out of scope). This requires the user to always store the handle in a persistent variable if the plotted graphics are to remain on the viewer.
    virtual GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color) = 0;
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

    /// Renders a 24bit RGB image of dimensions width and height from the current scene. The camera
    /// is meant to show the underlying OpenRAVE world as a robot would see it, so all graphs
    /// rendered with the plotX and drawX functions are hidden.
    /// \param pMemory the memory where the image will be stored at, has to store 3*width*height
    /// \param width width of the image
    /// \param height height of the image
    /// \param t the rotation and translation of the camera. Note that z is treated as the front of the camera!
    ///        So all points in front of the camera have a positive dot product with its direction.
    /// \param pKK 4 values such that the intrinsic matrix can be reconstructed [pKK[0] 0 pKK[2]; 0 pKK[1] pKK[3]; 0 0 1];
    virtual bool GetCameraImage(std::vector<uint8_t>& memory, int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK) = 0;

    /// Renders a 24bit RGB image of dimensions width and height from the current scene and saves it to a file.
    /// The camera is meant to show the underlying OpenRAVE world as a robot would see it, so all graphs
    /// rendered with the plotX and drawX functions are hidden.
    /// \param width width of the image
    /// \param height height of the image
    /// \param t the rotation and translation of the camera. Note that z is treated as the front of the camera!
    ///        So all points in front of the camera have a positive dot product with its direction.
    /// \param pKK 4 values such that the intrinsic matrix can be reconstructed [pKK[0] 0 pKK[2]; 0 pKK[1] pKK[3]; 0 0 1];
    virtual bool WriteCameraImage(int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK, const std::string& filename, const std::string& extension) = 0;
    //@}

    /// fill an array with all bodies loaded in the environment (including roobts)
    virtual void GetBodies(std::vector<KinBodyPtr>& bodies) const = 0;

    /// fill an array with all robots loaded in the environment
    virtual void GetRobots(std::vector<RobotBasePtr>& robots) const = 0;
    
    /// retrieve published bodies, note that the pbody pointer might become invalid
    /// as soon as GetPublishedBodies returns
    virtual void GetPublishedBodies(std::vector<KinBody::BodyState>& vbodies) = 0;

    /// updates the published bodies that viewers and other programs listening in on the environment see.
    /// For example, calling this function inside a planning loop allows the viewer to update the environment
    /// reflecting the status of the planner.
    /// Assumes that the physics are locked. 
    virtual void UpdatePublishedBodies() = 0;

    /// XML processing functions.
    //@{
    typedef boost::function<BaseXMLReaderPtr(InterfaceBasePtr, const std::list<std::pair<std::string,std::string> >&)> CreateXMLReaderFn;

    /// registers a custom xml reader for a particular interface. Once registered, anytime
    /// CreateXMLReaderFn(pinterface,atts)
    /// the tag specified in xmltag is seen in the interface, the the custom reader will be created.
    /// \return a pointer holding the registration, releasing the pointer will unregister the XML reader
    virtual boost::shared_ptr<void> RegisterXMLReader(PluginType type, const std::string& xmltag, const CreateXMLReaderFn& ) = 0;
    
    /// Parses a file for XML data
    virtual bool ParseXMLFile(BaseXMLReaderPtr preader, const std::string& filename) = 0;

    /// Parses a data file for XML data
    /// \param pdata The data of the buffer
    /// \param len the number of bytes valid in pdata
    virtual bool ParseXMLData(BaseXMLReaderPtr preader, const std::string& data) = 0;
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
    virtual bool Triangulate(KinBody::Link::TRIMESH& trimesh, KinBodyConstPtr pbody) = 0;

    /// general triangulation of the whole scene. trimesh will be appended the new data.
    /// \param opts - Controlls what to triangulate
    virtual bool TriangulateScene(KinBody::Link::TRIMESH& trimesh, TriangulateOptions opts, const std::string& name) = 0;

    /// returns the openrave home directory where settings, cache, and other files are stored.
    /// On Linux/Unix systems, this is usually $HOME/.openrave, on Windows this is $HOMEPATH/.openrave
    virtual const std::string& GetHomeDirectory() const = 0;

    //@{ debug/global commands
    
    /// sets the debug level
    /// \param level 0 for no debug, 1 - to print all debug messeges. Default
    ///             value for release builds is 0, for debug builds it is 1
    /// declaring variables with stdcall can be a little complex
    virtual void SetDebugLevel(DebugLevel level) = 0;
    virtual DebugLevel GetDebugLevel() const = 0;
    //@}
};

/// returns the a 16 character string specifying a hash of the interfaces used for checking changes
inline const char* RaveGetInterfaceHash(PluginType type)
{
    switch(type) {
    case PT_Planner: return OPENRAVE_PLANNER_HASH;
    case PT_Robot: return OPENRAVE_ROBOT_HASH;
    case PT_SensorSystem: return OPENRAVE_SENSORSYSTEM_HASH;
    case PT_Controller: return OPENRAVE_CONTROLLER_HASH;
    case PT_ProblemInstance: return OPENRAVE_PROBLEM_HASH;
    case PT_InverseKinematicsSolver: return OPENRAVE_IKSOLVER_HASH;
    case PT_KinBody: return OPENRAVE_KINBODY_HASH;
    case PT_PhysicsEngine: return OPENRAVE_PHYSICSENGINE_HASH;
    case PT_Sensor: return OPENRAVE_SENSOR_HASH;
    case PT_CollisionChecker: return OPENRAVE_COLLISIONCHECKER_HASH;
    case PT_Trajectory: return OPENRAVE_TRAJECTORY_HASH;
    case PT_Viewer: return OPENRAVE_VIEWER_HASH;
    default:
        throw openrave_exception("failed to find openrave interface type",ORE_InvalidArguments);
        return NULL;
    }
}

/// returns the a lower case string of the interface type
RAVE_API const std::map<PluginType,std::string>& RaveGetInterfaceNamesMap();
RAVE_API const std::string& RaveGetInterfaceName(PluginType type);

/// Random number generation
//@{
RAVE_API void RaveInitRandomGeneration(uint32_t seed);
/// generate a random integer, 32bit precision
RAVE_API uint32_t RaveRandomInt();
/// generate n random integers, 32bit precision
RAVE_API void RaveRandomInt(int n, std::vector<int>& v);
/// generate a random float in [0,1], 32bit precision
RAVE_API float RaveRandomFloat();
/// generate n random floats in [0,1], 32bit precision
RAVE_API void RaveRandomFloat(int n, std::vector<float>& v);
/// generate a random double in [0,1], 53bit precision
RAVE_API double RaveRandomDouble();
/// generate n random doubles in [0,1], 53bit precision
RAVE_API void RaveRandomDouble(int n, std::vector<double>& v);
//@}

} // end namespace OpenRAVE

#if !defined(RAVE_DISABLE_ASSERT_HANDLER) && defined(BOOST_ENABLE_ASSERT_HANDLER)
namespace boost
{
inline void assertion_failed(char const * expr, char const * function, char const * file, long line)
{
    throw OpenRAVE::openrave_exception(str(boost::format("[%s:%d] -> %s, expr: %s")%file%line%function%expr),OpenRAVE::ORE_Assert);
}
}
#endif

BOOST_STATIC_ASSERT(OPENRAVE_VERSION_MAJOR>=0&&OPENRAVE_VERSION_MAJOR<=255);
BOOST_STATIC_ASSERT(OPENRAVE_VERSION_MINOR>=0&&OPENRAVE_VERSION_MINOR<=255);
BOOST_STATIC_ASSERT(OPENRAVE_VERSION_PATCH>=0&&OPENRAVE_VERSION_PATCH<=255);

// register for typeof (MSVC only)
#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::PluginType)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::ProblemInstance)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::ControllerBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::PlannerBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::PlannerBase::PlannerParameters)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::IkSolverBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::SensorBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::SensorBase::SensorData)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::SensorSystemBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::SimpleSensorSystem)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::SimpleSensorSystem::XMLData)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::IkSolverBase)
BOOST_TYPEOF_REGISTER_TEMPLATE(OpenRAVE::RaveVector, 1)
BOOST_TYPEOF_REGISTER_TEMPLATE(OpenRAVE::RaveTransform, 1)
BOOST_TYPEOF_REGISTER_TEMPLATE(OpenRAVE::RaveTransformMatrix, 1)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::Joint)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::Link)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::Link::GEOMPROPERTIES)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::Link::TRIMESH)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::KinBodyStateSaver)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::BodyState)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::ManageData)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RobotBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RobotBase::Manipulator)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RobotBase::AttachedSensor)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RobotBase::GRABBED)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RobotBase::RobotStateSaver)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::TrajectoryBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::TrajectoryBase::TPOINT)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::TrajectoryBase::TSEGMENT)

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
