// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov (rosen.diankov@gmail.com)
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
/** \file rave.h
    \brief  Defines the public headers that every plugin must include in order to use openrave properly.
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

/// \brief openrave constant for PI, could be replaced by accurate precision number depending on choice of dReal.
static const dReal PI = (dReal)3.14159265358979323846;

/// Wrappers of common basic math functions, allows OpenRAVE to control the precision requirements.
/// \ingroup affine_math
//@{

/// \brief exponential
RAVE_API dReal RaveExp(dReal f);
/// \brief logarithm
RAVE_API dReal RaveLog(dReal f);
/// \brief cosine
RAVE_API dReal RaveCos(dReal f);
/// \brief sine
RAVE_API dReal RaveSin(dReal f);
/// \brief tangent
RAVE_API dReal RaveTan(dReal f);
/// \brief base 2 logarithm
RAVE_API dReal RaveLog2(dReal f);
/// \brief base 10 logarithm
RAVE_API dReal RaveLog10(dReal f);
/// \brief arccosine
RAVE_API dReal RaveAcos(dReal f);
/// \brief arcsine
RAVE_API dReal RaveAsin(dReal f);
/// \brief arctangent2 covering entire circle
RAVE_API dReal RaveAtan2(dReal fy, dReal fx);
/// \brief power x^y
RAVE_API dReal RavePow(dReal fx, dReal fy);
/// \brief square-root
RAVE_API dReal RaveSqrt(dReal f);
/// \brief absolute value
RAVE_API dReal RaveFabs(dReal f);

//@}

/// %OpenRAVE error codes
enum OpenRAVEErrorCode {
    ORE_Failed=0,
    ORE_InvalidArguments=1,
    ORE_EnvironmentNotLocked=2,
    ORE_CommandNotSupported=3, ///< string command could not be parsed or is not supported
    ORE_Assert=4,
    ORE_InvalidPlugin=5, ///< shared object is not a valid plugin
    ORE_InvalidInterfaceHash=6, ///< interface hashes do not match between plugins
    ORE_NotImplemented=7, ///< function is not implemented by the interface.
};

/// \brief Exception that all OpenRAVE internal methods throw; the error codes are held in \ref OpenRAVEErrorCode.
class openrave_exception : std::exception
{
public:
    openrave_exception() : std::exception(), _s("unknown exception"), _error(ORE_Failed) {}
    openrave_exception(const std::string& s, OpenRAVEErrorCode error=ORE_Failed) : std::exception() {
        _error = error;
        _s = "openrave (";
        switch(_error) {
        case ORE_Failed: _s += "Failed"; break;
        case ORE_InvalidArguments: _s += "InvalidArguments"; break;
        case ORE_EnvironmentNotLocked: _s += "EnvironmentNotLocked"; break;
        case ORE_CommandNotSupported: _s += "CommandNotSupported"; break;
        case ORE_Assert: _s += "Assert"; break;
        case ORE_InvalidPlugin: _s += "InvalidPlugin"; break;
        case ORE_InvalidInterfaceHash: _s += "InvalidInterfaceHash"; break;
        case ORE_NotImplemented: _s += "NotImplemented"; break;
        default:
            _s += boost::str(boost::format("%8.8x")%static_cast<int>(_error));
            break;
        }
        _s += "): ";
        _s += s; 
    }
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
            if(::toupper(*it1) != ::toupper(*it2)) { //letters differ?
                // return -1 to indicate 'smaller than', 1 otherwise
                return ::toupper(*it1) < ::toupper(*it2);
            }
            //proceed to the next character in each string
            ++it1;
            ++it2;
        }
        std::size_t size1=s1.size(), size2=s2.size();// cache lengths
        //return -1,0 or 1 according to strings' lengths
        if (size1==size2) {
            return 0;
        }
        return size1<size2;
    }
};

/// \brief base class for all user data
class UserData
{
 public:
    virtual ~UserData() {} 
};
typedef boost::shared_ptr<UserData> UserDataPtr;

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
    return command;
}

/// Change the text color (on either stdout or stderr) with an attr:fg (thanks to Radu Rusu for the code)
inline std::string ChangeTextColor (int attribute, int fg)
{
    char command[13];
    sprintf (command, "%c[%d;%dm", 0x1B, attribute, fg + 30);
    return command;
}

/// Reset the text color (on either stdout or stderr) to its original state (thanks to Radu Rusu for the code)
inline std::string ResetTextColor()
{
    char command[12];
    sprintf (command, "%c[0;38;48m", 0x1B);
    return command;
}

inline std::wstring ChangeTextColorW (int attribute, int fg)
{
    wchar_t command[13];
    swprintf (command, 13, L"%c[%d;%dm", 0x1B, attribute, fg + 30);
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

/// Random number generation
//@{
enum IntervalType {
    IT_Open=0, ///< (a,b)
    IT_OpenStart=1, ///< (a,b]
    IT_OpenEnd=2, ///< [a,b)
    IT_Closed=3, ///< [a,b]
};

RAVE_API void RaveInitRandomGeneration(uint32_t seed);
/// generate a random integer, 32bit precision
RAVE_API uint32_t RaveRandomInt();
/// generate n random integers, 32bit precision
RAVE_API void RaveRandomInt(int n, std::vector<int>& v);

/// \brief generate a random float in 0-1
///
/// \param interval specifies inclusion of 0 and 1 in the result
RAVE_API float RaveRandomFloat(IntervalType interval=IT_Closed);

/// \deprecated (10/11/27)
RAVE_API void RaveRandomFloat(int n, std::vector<float>& v) RAVE_DEPRECATED;

/// \brief generate a random double in 0-1, 53bit precision
///
/// \param interval specifies inclusion of 0 and 1 in the result
RAVE_API double RaveRandomDouble(IntervalType interval=IT_Closed);

/// \deprecated (10/11/27)
RAVE_API void RaveRandomDouble(int n, std::vector<double>& v) RAVE_DEPRECATED;
//@}

/// Sets the global openrave debug level
RAVE_API void RaveSetDebugLevel(DebugLevel level);

/// Returns the openrave debug level
RAVE_API DebugLevel RaveGetDebugLevel();

/// extracts only the filename
inline const char* RaveGetSourceFilename(const char* pfilename)
{
    if( pfilename == NULL ) {
        return "";
    }
    const char* p0 = strrchr(pfilename,'/');
    const char* p1 = strrchr(pfilename,'\\');
    const char* p = p0 > p1 ? p0 : p1;
    if( p == NULL ) {
        return pfilename;
    }
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
        if( s.size() == 0 || s[s.size()-1] != '\n' ) {  \
            printf("%s\n", s.c_str()); \
        } \
        else { \
            printf ("%s", s.c_str()); \
        } \
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
        /*if( fmt[0] != '\n' ) { printf("\n"); }*/  \
        /*ResetTextColor(stdout);*/ \
        return r; \
    }

inline int RavePrintfA(const std::string& s, DebugLevel level)
{
    if( s.size() == 0 || s[s.size()-1] != '\n' ) { // automatically add a new line
        printf("%s\n", s.c_str());
    }
    else {
        printf ("%s", s.c_str());
    }
    return s.size();
}

DefineRavePrintfW(_INFOLEVEL)
DefineRavePrintfA(_INFOLEVEL)

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
    inline int RavePrintfA_INFOLEVEL(const std::string& s)
    {
        if( s.size() == 0 || s[s.size()-1] != '\n' ) { // automatically add a new line
            printf("%s\n", s.c_str());
        }
        else {
            printf ("%s", s.c_str());
        }
        return s.size();
    }
    
    inline int RavePrintfA_INFOLEVEL(const char *fmt, ...)
    {
        va_list list;
	    va_start(list,fmt);
        int r = vprintf(fmt, list);
        va_end(list);
        //if( fmt[0] != '\n' ) { printf("\n"); }
        return r;
    }

#define DefineRavePrintfA(LEVEL) \
    inline int RavePrintfA##LEVEL(const std::string& s) \
    { \
        if( s.size() == 0 || s[s.size()-1] != '\n' ) { \
            printf ("%c[0;%d;%dm%s%c[m\n", 0x1B, OPENRAVECOLOR##LEVEL + 30,8+40,s.c_str(),0x1B); \
        } \
        else { \
            printf ("%c[0;%d;%dm%s%c[m", 0x1B, OPENRAVECOLOR##LEVEL + 30,8+40,s.c_str(),0x1B); \
        } \
        return s.size(); \
    } \
    \
    inline int RavePrintfA##LEVEL(const char *fmt, ...) \
    { \
        va_list list; \
	    va_start(list,fmt); \
        int r = vprintf((ChangeTextColor(0, OPENRAVECOLOR##LEVEL,8) + std::string(fmt) + ResetTextColor()).c_str(), list); \
        va_end(list); \
        /*if( fmt[0] != '\n' ) { printf("\n"); } */ \
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
        case Level_Info: // print regular
            if( s.size() == 0 || s[s.size()-1] != '\n' ) { // automatically add a new line
                printf ("%s\n",s.c_str());
            }
            else {
                printf ("%s",s.c_str());
            }
            return s.size(); 
        case Level_Debug: color = OPENRAVECOLOR_DEBUGLEVEL; break;
        case Level_Verbose: color = OPENRAVECOLOR_VERBOSELEVEL; break;
        }
        if( s.size() == 0 || s[s.size()-1] != '\n' ) { // automatically add a new line
            printf ("%c[0;%d;%dm%s%c[0;38;48m\n", 0x1B, color + 30,8+40,s.c_str(),0x1B);
        }
        else {
            printf ("%c[0;%d;%dm%s%c[0;38;48m", 0x1B, color + 30,8+40,s.c_str(),0x1B);
        }
        return s.size();
    }
    return 0;
}

#endif

DefineRavePrintfW(_FATALLEVEL)
DefineRavePrintfW(_ERRORLEVEL)
DefineRavePrintfW(_WARNLEVEL)
//DefineRavePrintfW(_INFOLEVEL)
DefineRavePrintfW(_DEBUGLEVEL)
DefineRavePrintfW(_VERBOSELEVEL)

DefineRavePrintfA(_FATALLEVEL)
DefineRavePrintfA(_ERRORLEVEL)
DefineRavePrintfA(_WARNLEVEL)
//DefineRavePrintfA(_INFOLEVEL)
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

/// \brief Enumeration of all the interfaces.
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
    PT_Viewer=12,///< describes \ref ViewerBase
    PT_NumberOfInterfaces=12 ///< number of interfaces, do not forget to update
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
class ViewerBase;
class IkParameterization;

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
typedef boost::shared_ptr<ViewerBase> ViewerBasePtr;
typedef boost::shared_ptr<ViewerBase const> ViewerBaseConstPtr;
typedef boost::weak_ptr<ViewerBase> ViewerBaseWeakPtr;
typedef boost::shared_ptr<EnvironmentBase> EnvironmentBasePtr;
typedef boost::shared_ptr<EnvironmentBase const> EnvironmentBaseConstPtr;
typedef boost::weak_ptr<EnvironmentBase> EnvironmentBaseWeakPtr;

///< Cloning Options for interfaces and environments
enum CloningOptions {
    Clone_Bodies = 1, ///< clone all the bodies/robots of the environment, exclude attached interfaces like sensors/controllers
    Clone_Viewer = 2, ///< clone the viewer type, although figures won't be copied, new viewer does try to match views
    Clone_Simulation = 4, ///< clone the physics engine and simulation state (ie, timesteps, gravity)
    Clone_RealControllers = 8, ///< if specified, will clone the real controllers of all the robots, otherwise each robot gets ideal controller
    Clone_Sensors = 16, ///< if specified, will clone the sensors attached to the robot and added to the environment
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
typedef std::list<std::pair<std::string,std::string> > XMLAttributesList;

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

typedef boost::function<BaseXMLReaderPtr(InterfaceBasePtr, const std::list<std::pair<std::string,std::string> >&)> CreateXMLReaderFn;

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

// define the math functions
#define MATH_EXP RaveExp
#define MATH_LOG RaveLog
#define MATH_COS RaveCos
#define MATH_SIN RaveSin
#define MATH_TAN RaveTan
#define MATH_LOG2 RaveLog2
#define MATH_LOG10 RaveLog10
#define MATH_ACOS RaveAcos
#define MATH_ASIN RaveAsin
#define MATH_ATAN2 RaveAtan2
#define MATH_POW RavePow
#define MATH_SQRT RaveSqrt
#define MATH_FABS RaveFabs

#include <rave/geometry.h>
#include <rave/mathextra.h>

namespace OpenRAVE {
    using geometry::RaveVector;
    using geometry::RaveTransform;
    using geometry::RaveTransformMatrix;
    typedef RaveVector<dReal> Vector;
    typedef RaveTransform<dReal> Transform;
    typedef boost::shared_ptr< RaveTransform<dReal> > TransformPtr;
    typedef boost::shared_ptr< RaveTransform<dReal> const > TransformConstPtr;
    typedef RaveTransformMatrix<dReal> TransformMatrix;
    typedef boost::shared_ptr< RaveTransformMatrix<dReal> > TransformMatrixPtr;
    typedef boost::shared_ptr< RaveTransformMatrix<dReal> const > TransformMatrixConstPtr;
    typedef geometry::obb<dReal> OBB;
    typedef geometry::aabb<dReal> AABB;
    typedef geometry::ray<dReal> RAY;
    // for compatibility
    //@{
    using mathextra::dot2;
    using mathextra::dot3;
    using mathextra::dot4;
    using mathextra::normalize2;
    using mathextra::normalize3;
    using mathextra::normalize4;
    using mathextra::cross3;
    using mathextra::inv3;
    using mathextra::inv4;
    using mathextra::lengthsqr2;
    using mathextra::lengthsqr3;
    using mathextra::lengthsqr4;
    using mathextra::mult4;
    //@}
}

#include <rave/plugininfo.h>
#include <rave/interface.h>
#include <rave/kinbody.h>
#include <rave/trajectory.h>
#include <rave/problems.h>
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

/// \name Global Functionality - Interface Creation, Plugin Management, Logging
/// \anchor global_functionality
//@{

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

/// \brief returns a lower case string of the interface type
RAVE_API const std::map<InterfaceType,std::string>& RaveGetInterfaceNamesMap();
RAVE_API const std::string& RaveGetInterfaceName(InterfaceType type);

/// \brief Returns the openrave home directory where settings, cache, and other files are stored.
///
/// On Linux/Unix systems, this is usually $HOME/.openrave, on Windows this is $HOMEPATH/.openrave
RAVE_API std::string RaveGetHomeDirectory();

/// \brief Searches for a filename in the database and returns a full path/URL to it
///
/// \param filename the relative filename in the database
/// \param bRead if true will only return a file if it exists. If false, will return the filename of the first valid database directory.
/// \return a non-empty string if a file could be found.
RAVE_API std::string RaveFindDatabaseFile(const std::string& filename, bool bRead=true);

/// \brief Explicitly initializes the global OpenRAVE state (optional).
///
/// Optional function to initialize openrave plugins and logging.
/// Although environment creation will automatically make sure this function is called, users might want
/// explicit control of when this happens.
/// \param bLoadAllPlugins If true will load all the openrave plugins automatically that can be found in the OPENRAVE_PLUGINS environment path
/// \return 0 if successful, otherwise an error code
RAVE_API int RaveInitialize(bool bLoadAllPlugins=true, DebugLevel level = Level_Info);

/// \brief Initializes the global state from an already loaded OpenRAVE environment.
///
/// Because of shared object boundaries, it is necessary to pass the global state pointer
/// around. If using plugin.h, this function is automatically called by \ref CreateInterfaceValidated.
/// It is also called by and every InterfaceBase constructor.
/// \param[in] globalstate 
RAVE_API void RaveInitializeFromState(UserDataPtr globalstate);

/// \brief A pointer to the global openrave state
/// \return a managed pointer to the state.
RAVE_API UserDataPtr RaveGlobalState();

/// \brief Destroys the entire OpenRAVE state and all loaded environments. 
///
/// This functions should be always called before program shutdown in order to assure all
/// resources are relased appropriately.
RAVE_API void RaveDestroy();

/// \brief Get all the loaded plugins and the interfaces they support.
///
/// \param plugins A list of plugins. Each entry has the plugin name and the interfaces it supports
RAVE_API void RaveGetPluginInfo(std::list< std::pair<std::string, PLUGININFO> >& plugins);

/// \brief Get a list of all the loaded interfaces.
RAVE_API void RaveGetLoadedInterfaces(std::map<InterfaceType, std::vector<std::string> >& interfacenames);

/// \brief Reloads all the plugins.
///
/// The interfaces currently created remain will continue using the old plugins, so this function is safe in that plugins currently loaded remain loaded until the last interface that uses them is released.
RAVE_API void RaveReloadPlugins();

/// \brief Load a plugin and its interfaces.
///
/// If the plugin is already loaded, will reload it.
/// \param name the filename of the plugin to load
RAVE_API bool RaveLoadPlugin(const std::string& libraryname);

/// \brief Returns true if interface can be created, otherwise false.
RAVE_API bool RaveHasInterface(InterfaceType type, const std::string& interfacename);

RAVE_API InterfaceBasePtr RaveCreateInterface(EnvironmentBasePtr penv, InterfaceType type,const std::string& interfacename);
RAVE_API RobotBasePtr RaveCreateRobot(EnvironmentBasePtr penv, const std::string& name="");
RAVE_API PlannerBasePtr RaveCreatePlanner(EnvironmentBasePtr penv, const std::string& name);
RAVE_API SensorSystemBasePtr RaveCreateSensorSystem(EnvironmentBasePtr penv, const std::string& name);
RAVE_API ControllerBasePtr RaveCreateController(EnvironmentBasePtr penv, const std::string& name);
RAVE_API ProblemInstancePtr RaveCreateProblem(EnvironmentBasePtr penv, const std::string& name);
RAVE_API IkSolverBasePtr RaveCreateIkSolver(EnvironmentBasePtr penv, const std::string& name);
RAVE_API PhysicsEngineBasePtr RaveCreatePhysicsEngine(EnvironmentBasePtr penv, const std::string& name);
RAVE_API SensorBasePtr RaveCreateSensor(EnvironmentBasePtr penv, const std::string& name);
RAVE_API CollisionCheckerBasePtr RaveCreateCollisionChecker(EnvironmentBasePtr penv, const std::string& name);
RAVE_API ViewerBasePtr RaveCreateViewer(EnvironmentBasePtr penv, const std::string& name);
RAVE_API  KinBodyPtr RaveCreateKinBody(EnvironmentBasePtr penv, const std::string& name="");
/// \brief Return an empty trajectory instance initialized to nDOF degrees of freedom. Will be deprecated soon
RAVE_API TrajectoryBasePtr RaveCreateTrajectory(EnvironmentBasePtr penv, int nDOF);
/// \brief Return an empty trajectory instance.
RAVE_API TrajectoryBasePtr RaveCreateTrajectory(EnvironmentBasePtr penv, const std::string& name="");

/** \brief Registers a function to create an interface, this allows the interface to be created by other modules.

    \param type interface type
    \param name interface name
    \param interfacehash the hash of the interface being created (use the global defines OPENRAVE_X_HASH)
    \param envhash the hash of the environment (use the global define OPENRAVE_ENVIRONMENT_HASH)
    \param createfn functions to create the interface it takes two parameters: the environment and an istream to the rest of the interface creation arguments.
    \return a handle if function is successfully registered. By destroying the handle, the interface will be automatically unregistered.
    \throw openrave_exception Will throw with ORE_InvalidInterfaceHash if hashes do not match
 */
RAVE_API boost::shared_ptr<void> RaveRegisterInterface(InterfaceType type, const std::string& name, const char* interfacehash, const char* envhash, const boost::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)>& createfn);

/** \brief Registers a custom xml reader for a particular interface.
    
    Once registered, anytime an interface is created through XML and
    the xmltag is seen, the function CreateXMLReaderFn will be called to get a reader for that tag
    \param xmltag the tag specified in xmltag is seen in the interface, the the custom reader will be created.
    \param fn CreateXMLReaderFn(pinterface,atts) - passed in the pointer to the interface where the tag was seen along with the list of attributes
    \return a pointer holding the registration, releasing the pointer will unregister the XML reader
*/
RAVE_API boost::shared_ptr<void> RaveRegisterXMLReader(InterfaceType type, const std::string& xmltag, const CreateXMLReaderFn& fn);

/// \brief return the environment's unique id, returns 0 if environment could not be found or not registered
RAVE_API int RaveGetEnvironmentId(EnvironmentBasePtr penv);

/// \brief get the environment from its unique id
/// \param id the unique environment id returned by \ref RaveGetEnvironmentId
RAVE_API EnvironmentBasePtr RaveGetEnvironment(int id);

/// \brief Return all the created OpenRAVE environments.
RAVE_API void RaveGetEnvironments(std::list<EnvironmentBasePtr>& listenvironments);

/// \brief Returns the current registered reader for the interface type/xmlid
///
/// \throw openrave_exception Will throw with ORE_InvalidArguments if registered function could not be found.
RAVE_API BaseXMLReaderPtr RaveCallXMLReader(InterfaceType type, const std::string& xmltag, InterfaceBasePtr pinterface, const std::list<std::pair<std::string,std::string> >& atts);

//@}

/// \brief separates the directories from a string and returns them in a vector
inline bool RaveParseDirectories(const char* pdirs, std::vector<std::string>& vdirs)
{
    vdirs.resize(0);
    if( !pdirs ) {
        return false;
    }
    // search for all directories separated by ':'
    std::string tmp = pdirs;
    std::string::size_type pos = 0, newpos=0;
    while( pos < tmp.size() ) {
#ifdef _WIN32
        newpos = tmp.find(';', pos);
#else
		newpos = tmp.find(':', pos);
#endif
        std::string::size_type n = newpos == std::string::npos ? tmp.size()-pos : (newpos-pos);
        vdirs.push_back(tmp.substr(pos, n));
        if( newpos == std::string::npos ) {
            break;
        }
        pos = newpos+1;
    }
    return true;
}

/// \brief Create the interfaces, see \ref CreateInterfaceValidated.
/// \ingroup plugin_exports
typedef InterfaceBasePtr (*PluginExportFn_OpenRAVECreateInterface)(InterfaceType type, const std::string& name, const char* pluginhash, const char* envhash, EnvironmentBasePtr penv);

/// \brief Called to fill information about the plugin, see \ref GetPluginAttributesValidated.
/// \ingroup plugin_exports
typedef bool (*PluginExportFn_OpenRAVEGetPluginAttributes)(PLUGININFO* pinfo, int size, const char* infohash);

/// \brief Called before plugin is unloaded from openrave. See \ref DestroyPlugin. 
/// \ingroup plugin_exports
typedef void (*PluginExportFn_DestroyPlugin)();

/// \deprecated
typedef InterfaceBasePtr (*PluginExportFn_CreateInterface)(InterfaceType type, const std::string& name, const char* pluginhash, EnvironmentBasePtr penv);

/// \deprecated
typedef bool (*PluginExportFn_GetPluginAttributes)(PLUGININFO* pinfo, int size);

} // end namespace OpenRAVE

#if !defined(RAVE_DISABLE_ASSERT_HANDLER) && defined(BOOST_ENABLE_ASSERT_HANDLER)
/// Modifications controlling %boost library behavior.
namespace boost
{
inline void assertion_failed(char const * expr, char const * function, char const * file, long line)
{
    throw OpenRAVE::openrave_exception(boost::str(boost::format("[%s:%d] -> %s, expr: %s")%file%line%function%expr),OpenRAVE::ORE_Assert);
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
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::ViewerBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::GraphHandle)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::IkParameterization)
BOOST_TYPEOF_REGISTER_TEMPLATE(OpenRAVE::RaveVector, 1)
BOOST_TYPEOF_REGISTER_TEMPLATE(OpenRAVE::RaveTransform, 1)
BOOST_TYPEOF_REGISTER_TEMPLATE(OpenRAVE::RaveTransformMatrix, 1)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::Joint)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::Joint::MIMIC)
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
