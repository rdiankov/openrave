// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rdiankov@cs.cmu.edu)
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
/**
\htmlonly
\file   rave.h
\brief  Defines the public headers that every plugin must include in order to use openrave properly.
\endhtmlonly
*/
#ifndef OPENRAVE_H
#define OPENRAVE_H

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
#pragma warning(disable:4819) //The file contains a character that cannot be represented in the current code page (932). Save the file in Unicode format to prevent data loss using native typeof

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
#include <boost/multi_array.hpp>

//#include <boost/cstdint.hpp>

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

#if defined(__GNUC__)
#define RAVE_DEPRECATED __attribute__((deprecated))
#else
#define RAVE_DEPRECATED
#endif

#define MATH_RANDOM_FLOAT RaveRandomFloat()

/// The entire %OpenRAVE library
namespace OpenRAVE {
    
#include <rave/defines.h>
#include <rave/classhashes.h>

#if OPENRAVE_PRECISION // 1 if double precision
typedef double dReal;
#define g_fEpsilon 1e-15
#else
typedef float dReal;
#define g_fEpsilon 2e-7f
#endif

#define PI ((dReal)3.14159265358979)

/// %OpenRAVE error codes
enum OpenRAVEErrorCode {
    ORE_Failed=0,
    ORE_InvalidArguments=1,
    ORE_EnvironmentNotLocked=2,
    ORE_CommandNotSupported=3,
    ORE_Assert=4,
    ORE_InvalidPlugin=5, ///< shared object is not a valid plugin
    ORE_InvalidInterfaceHash=6, ///< interface hashes do not match between plugins
};

struct openrave_exception : std::exception
{
    openrave_exception() : std::exception(), _s("unknown exception"), _error(ORE_Failed) {}
    openrave_exception(const std::string& s, OpenRAVEErrorCode error=ORE_Failed) : std::exception() { _s = "OpenRAVE: " + s; _error = error; }
    virtual ~openrave_exception() throw() {}
    char const* what() const throw() { return _s.c_str(); }
    const std::string& message() const { return _s; }
    OpenRAVEErrorCode GetCode() const { return _error; }
private:
    std::string _s;
    OpenRAVEErrorCode _error;
};

class CaseInsensitiveCompare
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

RAVE_API void RaveSetDebugLevel(DebugLevel level);

inline DebugLevel RaveGetDebugLevel(void) {
#ifdef RAVE_LIBBUILD
    extern DebugLevel g_nDebugLevel;
#else
    RAVE_API extern DebugLevel g_nDebugLevel;
#endif
    return g_nDebugLevel;
}

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

inline int RavePrintfA(const std::string& s, DebugLevel level)
{
    printf ("%s", s.c_str());
    return s.size();
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


inline int RavePrintfA(const std::string& s, DebugLevel level)
{
    if( OpenRAVE::RaveGetDebugLevel()>=level ) {
        int color = 0;
        switch(level) {
            case Level_Fatal: color = OPENRAVECOLOR_FATALLEVEL; break;
            case Level_Error: color = OPENRAVECOLOR_ERRORLEVEL; break;
            case Level_Warn: color = OPENRAVECOLOR_WARNLEVEL; break;
            case Level_Info: color = OPENRAVECOLOR_INFOLEVEL; break;
            case Level_Debug: color = OPENRAVECOLOR_DEBUGLEVEL; break;
            case Level_Verbose: color = OPENRAVECOLOR_VERBOSELEVEL; break;
        }
        printf ("%c[0;%d;%dm%s%c[0;38;48m", 0x1B, color + 30,8+40,s.c_str(),0x1B);
        return s.size();
    }
    return 0;
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

enum InterfaceType
{
    PT_Planner=1, ///< describes \ref PlannerBase interface
    PT_Robot=2, ///< describes \ref RobotBase interface
    PT_SensorSystem=3, ///< describes \ref SensorSystemBase interface
    PT_Controller=4, ///< describes \ref ControllerBase interface
    PT_ProblemInstance=5, ///< describes \ref ProblemInstance interface
    PT_InverseKinematicsSolver=6, ///< describes \ref IkSolverBase interface
    PT_KinBody=7, ///< describes \ref KinBody
    PT_PhysicsEngine=8, ///< describes \ref PhysicsEngineBase
    PT_Sensor=9, ///< describes \ref SensorBase
    PT_CollisionChecker=10, ///< describes \ref CollisionCheckerBase
    PT_Trajectory=11, ///< describes \ref TrajectoryBase
    PT_Viewer=12,///< describes \ref RaveViewerBase
};

typedef InterfaceType PluginType RAVE_DEPRECATED;

class CollisionReport;
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

typedef boost::shared_ptr<CollisionReport> CollisionReportPtr;
typedef boost::shared_ptr<CollisionReport const> CollisionReportConstPtr;
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
    Clone_RealControllers = 8, ///< if specified, will clone the real controllers of all the robots, otherwise each robot gets ideal controller
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
/// xml files. Custom readers can be registered through EnvironmentBase.
/// By default it can record all data that is encountered inside the xml reader
class RAVE_API BaseXMLReader : public boost::enable_shared_from_this<BaseXMLReader>
{
public:
    enum ProcessElement
    {
        PE_Pass=0, ///< current tag was not supported, so pass onto another class
        PE_Support=1, ///< current tag will be processed by this class
        PE_Ignore=2, ///< current tag and all its children should be ignored
    };
    BaseXMLReader() {}
    virtual ~BaseXMLReader() {}

    /// a readable interface that stores the information processsed for the current tag
    /// This pointer is used to the InterfaceBase class registered readers
    virtual XMLReadablePtr GetReadable() { return XMLReadablePtr(); }

    /// Gets called in the beginning of each "<type>" expression. In this case, name is "type"
    /// \param name of the tag, will be always lower case
    /// \param atts string of attributes where the first std::string is the attribute name and second is the value
    /// \return true if tag is accepted and this class will process it, otherwise false
    virtual ProcessElement startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts) = 0;

    /// Gets called at the end of each "</type>" expression. In this case, name is "type"
    /// \param name of the tag, will be always lower case
    /// \return true if XMLReader has finished parsing (one condition is that name==_fieldname) , otherwise false
    virtual bool endElement(const std::string& name) = 0;

    /// gets called for all data in between tags.
    /// \param ch a string to the data
    virtual void characters(const std::string& ch) = 0;

    /// XML filename/resource used for this class (can be empty)
    std::string _filename;
};

typedef boost::shared_ptr<BaseXMLReader> BaseXMLReaderPtr;
typedef boost::shared_ptr<BaseXMLReader const> BaseXMLReaderConstPtr;

/// reads until the tag ends
class RAVE_API DummyXMLReader : public BaseXMLReader
{
public:
    DummyXMLReader(const std::string& pfieldname, const std::string& pparentname, boost::shared_ptr<std::ostream> osrecord = boost::shared_ptr<std::ostream>());
    virtual ProcessElement startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts);
    virtual bool endElement(const std::string& name);
    virtual void characters(const std::string& ch);
    const std::string& GetFieldName() const { return _fieldname; }
private:
    std::string _parentname; /// XML filename
    std::string _fieldname;
    boost::shared_ptr<std::ostream> _osrecord; ///< used to store the xml data
    boost::shared_ptr<BaseXMLReader> _pcurreader;
};

} // end namespace OpenRAVE


#include <rave/geometry.h>

namespace OpenRAVE {
    using geometry::RaveVector;
    using geometry::RaveTransform;
    using geometry::RaveTransformMatrix;
    using geometry::RaveSqrt;
    using geometry::RaveSin;
    using geometry::RaveCos;
    using geometry::RaveFabs;
    using geometry::RaveAcos;
    using geometry::RaveAsin;
    using geometry::RaveAtan2;
    using geometry::aabb;
    using geometry::obb;
    using geometry::ray;
    typedef RaveVector<dReal> Vector;
    typedef RaveTransform<dReal> Transform;
    typedef RaveTransformMatrix<dReal> TransformMatrix;
    typedef obb<dReal> OBB;
    typedef aabb<dReal> AABB;
    typedef ray<dReal> RAY;
}

#include <rave/plugininfo.h>
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
#include <rave/environment.h>

namespace OpenRAVE {

/// \brief Returns the a 16 character null-terminated string specifying a hash of the interfaces used for checking changes.
inline const char* RaveGetInterfaceHash(InterfaceType type)
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

/// safely casts from the base interface class to an openrave interface using static_pointer_cast.
/// The reason why dynamic_pointer_cast cannot be used is because interfaces might be created by different plugins, and the runtime type information will be different.
template <typename T>
inline boost::shared_ptr<T> RaveInterfaceCast(InterfaceBasePtr pinterface)
{
    if( !!pinterface ) {
        if( pinterface->GetInterfaceType() == T::GetInterfaceTypeStatic() ) {
            return boost::static_pointer_cast<T>(pinterface);
        }
        // encode special cases
        if( pinterface->GetInterfaceType() == PT_Robot && T::GetInterfaceTypeStatic() == PT_KinBody ) {
            return boost::static_pointer_cast<T>(pinterface);
        }
    }
    return boost::shared_ptr<T>();
}

/// safely casts from the base interface class to an openrave interface using static_pointer_cast.
/// The reason why dynamic_pointer_cast cannot be used is because interfaces might be created by different plugins, and the runtime type information will be different.
template <typename T>
inline boost::shared_ptr<T const> RaveInterfaceConstCast(InterfaceBaseConstPtr pinterface)
{
    if( !!pinterface ) {
        if( pinterface->GetInterfaceType() == T::GetInterfaceTypeStatic() ) {
            return boost::static_pointer_cast<T const>(pinterface);
        }
        // encode special cases
        if( pinterface->GetInterfaceType() == PT_Robot && T::GetInterfaceTypeStatic() == PT_KinBody ) {
            return boost::static_pointer_cast<T const>(pinterface);
        }
    }
    return boost::shared_ptr<T>();
}

/// returns the a lower case string of the interface type
RAVE_API const std::map<InterfaceType,std::string>& RaveGetInterfaceNamesMap();
RAVE_API const std::string& RaveGetInterfaceName(InterfaceType type);

/// returns the openrave home directory where settings, cache, and other files are stored.
/// On Linux/Unix systems, this is usually $HOME/.openrave, on Windows this is $HOMEPATH/.openrave
RAVE_API std::string RaveGetHomeDirectory();

/// create the interfaces
typedef InterfaceBasePtr (*PluginExportFn_CreateInterface)(InterfaceType type, const std::string& name, const char* pluginhash, EnvironmentBasePtr penv);

/// \brief Create an interface.
/// \ingroup plugin_exports
typedef InterfaceBasePtr (*PluginExportFn_OpenRAVECreateInterface)(InterfaceType type, const std::string& name, const char* pluginhash, const char* envhash, EnvironmentBasePtr penv);

/** \brief Called to fill information about the plugin.
    
    \ingroup plugin_exports
    This function is called only once initially to determine what the plugin offers. It should be
    the safest funcdtion and should not create any static resources for the plugin.
*/
typedef bool (*PluginExportFn_OpenRAVEGetPluginAttributes)(PLUGININFO* pinfo, int size, const char* infohash);

typedef bool (*PluginExportFn_GetPluginAttributes)(PLUGININFO* pinfo, int size);

/// \brief Called before plugin is unloaded from openrave.
/// \ingroup plugin_exports
typedef void (*PluginExportFn_DestroyPlugin)();

} // end namespace OpenRAVE

#if !defined(RAVE_DISABLE_ASSERT_HANDLER) && defined(BOOST_ENABLE_ASSERT_HANDLER)
/// Modifications controlling %boost library behavior.
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
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::InterfaceType)

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
