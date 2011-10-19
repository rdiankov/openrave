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
/** \file openrave.h
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

#if defined(__GNUC__)
#define RAVE_DEPRECATED __attribute__((deprecated))
#else
#define RAVE_DEPRECATED
#endif

/// The entire %OpenRAVE library
namespace OpenRAVE {

#include <openrave/config.h>
#include <openrave/interfacehashes.h>

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
OPENRAVE_API dReal RaveExp(dReal f);
/// \brief logarithm
OPENRAVE_API dReal RaveLog(dReal f);
/// \brief cosine
OPENRAVE_API dReal RaveCos(dReal f);
/// \brief sine
OPENRAVE_API dReal RaveSin(dReal f);
/// \brief tangent
OPENRAVE_API dReal RaveTan(dReal f);
/// \brief base 2 logarithm
OPENRAVE_API dReal RaveLog2(dReal f);
/// \brief base 10 logarithm
OPENRAVE_API dReal RaveLog10(dReal f);
/// \brief arccosine
OPENRAVE_API dReal RaveAcos(dReal f);
/// \brief arcsine
OPENRAVE_API dReal RaveAsin(dReal f);
/// \brief arctangent2 covering entire circle
OPENRAVE_API dReal RaveAtan2(dReal fy, dReal fx);
/// \brief power x^y
OPENRAVE_API dReal RavePow(dReal fx, dReal fy);
/// \brief square-root
OPENRAVE_API dReal RaveSqrt(dReal f);
/// \brief absolute value
OPENRAVE_API dReal RaveFabs(dReal f);

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
    ORE_InconsistentConstraints=8, ///< return solutions or trajectories do not follow the constraints of the planner/module
};

/// \brief Exception that all OpenRAVE internal methods throw; the error codes are held in \ref OpenRAVEErrorCode.
class OPENRAVE_API openrave_exception : public std::exception
{
public:
    openrave_exception() : std::exception(), _s("unknown exception"), _error(ORE_Failed) {
    }
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
    virtual ~openrave_exception() throw() {
    }
    char const* what() const throw() {
        return _s.c_str();
    }
    const std::string& message() const {
        return _s;
    }
    OpenRAVEErrorCode GetCode() const {
        return _error;
    }
private:
    std::string _s;
    OpenRAVEErrorCode _error;
};

class OPENRAVE_LOCAL CaseInsensitiveCompare
{
public:
    bool operator() (const std::string & s1, const std::string& s2) const
    {
        std::string::const_iterator it1=s1.begin();
        std::string::const_iterator it2=s2.begin();

        //has the end of at least one of the strings been reached?
        while ( (it1!=s1.end()) && (it2!=s2.end()) )  {
            if(::toupper(*it1) != ::toupper(*it2)) {     //letters differ?
                // return -1 to indicate 'smaller than', 1 otherwise
                return ::toupper(*it1) < ::toupper(*it2);
            }
            //proceed to the next character in each string
            ++it1;
            ++it2;
        }
        std::size_t size1=s1.size(), size2=s2.size();     // cache lengths
        //return -1,0 or 1 according to strings' lengths
        if (size1==size2) {
            return 0;
        }
        return size1<size2;
    }
};

/// \brief base class for all user data
class OPENRAVE_API UserData
{
public:
    virtual ~UserData() {
    }
};
typedef boost::shared_ptr<UserData> UserDataPtr;
typedef boost::weak_ptr<UserData> UserDataWeakPtr;

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
    Level_Verbose=5,
    Level_OutputMask=0xf,
    Level_VerifyPlans=0x80000000, ///< if set, should verify every plan returned. the verification is left up to the planners or the modules calling the planners. See \ref planningutils::ValidateTrajectory
};

#define OPENRAVECOLOR_FATALLEVEL 5 // magenta
#define OPENRAVECOLOR_ERRORLEVEL 1 // red
#define OPENRAVECOLOR_WARNLEVEL 3 // yellow
#define OPENRAVECOLOR_INFOLEVEL 0 // black
#define OPENRAVECOLOR_DEBUGLEVEL 2 // green
#define OPENRAVECOLOR_VERBOSELEVEL 4 // blue

/// \brief Sets the global openrave debug level. A combination of \ref DebugLevel
OPENRAVE_API void RaveSetDebugLevel(int level);

/// Returns the openrave debug level
OPENRAVE_API int RaveGetDebugLevel();

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
    inline int RavePrintfW ## LEVEL(const wchar_t *fmt, ...) \
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
    inline int RavePrintfA ## LEVEL(const std::string& s) \
    { \
        if((s.size() == 0)||(s[s.size()-1] != '\n')) {  \
            printf("%s\n", s.c_str()); \
        } \
        else { \
            printf ("%s", s.c_str()); \
        } \
        return s.size(); \
    } \
    \
    inline int RavePrintfA ## LEVEL(const char *fmt, ...) \
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

inline int RavePrintfA(const std::string& s, uint32_t level)
{
    if((s.size() == 0)||(s[s.size()-1] != '\n')) { // automatically add a new line
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
    inline int RavePrintfW ## LEVEL(const wchar_t *wfmt, ...) \
    { \
        va_list list; \
        va_start(list,wfmt); \
        /* Allocate memory on the stack to avoid heap fragmentation */ \
        size_t allocsize = wcstombs(NULL, wfmt, 0)+32; \
        char* fmt = (char*)alloca(allocsize); \
        strcpy(fmt, ChangeTextColor(0, OPENRAVECOLOR ## LEVEL,8).c_str()); \
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
    if((s.size() == 0)||(s[s.size()-1] != '\n')) {     // automatically add a new line
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
    inline int RavePrintfA ## LEVEL(const std::string& s) \
    { \
        if((s.size() == 0)||(s[s.size()-1] != '\n')) { \
            printf ("%c[0;%d;%dm%s%c[m\n", 0x1B, OPENRAVECOLOR ## LEVEL + 30,8+40,s.c_str(),0x1B); \
        } \
        else { \
            printf ("%c[0;%d;%dm%s%c[m", 0x1B, OPENRAVECOLOR ## LEVEL + 30,8+40,s.c_str(),0x1B); \
        } \
        return s.size(); \
    } \
    \
    inline int RavePrintfA ## LEVEL(const char *fmt, ...) \
    { \
        va_list list; \
        va_start(list,fmt); \
        int r = vprintf((ChangeTextColor(0, OPENRAVECOLOR ## LEVEL,8) + std::string(fmt) + ResetTextColor()).c_str(), list); \
        va_end(list); \
        /*if( fmt[0] != '\n' ) { printf("\n"); } */ \
        return r; \
    } \


inline int RavePrintfA(const std::string& s, uint32_t level)
{
    if( (OpenRAVE::RaveGetDebugLevel()&OpenRAVE::Level_OutputMask)>=level ) {
        int color = 0;
        switch(level) {
        case Level_Fatal: color = OPENRAVECOLOR_FATALLEVEL; break;
        case Level_Error: color = OPENRAVECOLOR_ERRORLEVEL; break;
        case Level_Warn: color = OPENRAVECOLOR_WARNLEVEL; break;
        case Level_Info: // print regular
            if((s.size() == 0)||(s[s.size()-1] != '\n')) { // automatically add a new line
                printf ("%s\n",s.c_str());
            }
            else {
                printf ("%s",s.c_str());
            }
            return s.size();
        case Level_Debug: color = OPENRAVECOLOR_DEBUGLEVEL; break;
        case Level_Verbose: color = OPENRAVECOLOR_VERBOSELEVEL; break;
        }
        if((s.size() == 0)||(s[s.size()-1] != '\n')) { // automatically add a new line
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

#define RAVEPRINTHEADER(LEVEL) OpenRAVE::RavePrintfA ## LEVEL("[%s:%d] ", OpenRAVE::RaveGetSourceFilename(__FILE__), __LINE__)

// different logging levels. The higher the suffix number, the less important the information is.
// 0 log level logs all the time. OpenRAVE starts up with a log level of 0.
#define RAVELOG_LEVELW(LEVEL,level) (OpenRAVE::RaveGetDebugLevel()&OpenRAVE::Level_OutputMask)>=(level)&&(RAVEPRINTHEADER(LEVEL)>0)&&OpenRAVE::RavePrintfW ## LEVEL
#define RAVELOG_LEVELA(LEVEL,level) (OpenRAVE::RaveGetDebugLevel()&OpenRAVE::Level_OutputMask)>=(level)&&(RAVEPRINTHEADER(LEVEL)>0)&&OpenRAVE::RavePrintfA ## LEVEL

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

#define IS_DEBUGLEVEL(level) ((OpenRAVE::RaveGetDebugLevel()&OpenRAVE::Level_OutputMask)>=(level))

#define OPENRAVE_EXCEPTION_FORMAT0(s, errorcode) OpenRAVE::openrave_exception(boost::str(boost::format("[%s:%d] " s)%(__PRETTY_FUNCTION__)%(__LINE__)),errorcode)

/// adds the function name and line number to an openrave exception
#define OPENRAVE_EXCEPTION_FORMAT(s, args,errorcode) OpenRAVE::openrave_exception(boost::str(boost::format("[%s:%d] " s)%(__PRETTY_FUNCTION__)%(__LINE__)%args),errorcode)

#define OPENRAVE_DUMMY_IMPLEMENTATION { throw OPENRAVE_EXCEPTION_FORMAT0("not implemented",ORE_NotImplemented); }

/// \brief Enumeration of all the interfaces.
enum InterfaceType
{
    PT_Planner=1, ///< describes \ref PlannerBase interface
    PT_Robot=2, ///< describes \ref RobotBase interface
    PT_SensorSystem=3, ///< describes \ref SensorSystemBase interface
    PT_Controller=4, ///< describes \ref ControllerBase interface
    PT_Module=5, ///< describes \ref ModuleBase interface
    PT_ProblemInstance=5, ///< describes \ref ModuleBase interface
    PT_IkSolver=6, ///< describes \ref IkSolverBase interface
    PT_InverseKinematicsSolver=6, ///< describes \ref IkSolverBase interface
    PT_KinBody=7, ///< describes \ref KinBody
    PT_PhysicsEngine=8, ///< describes \ref PhysicsEngineBase
    PT_Sensor=9, ///< describes \ref SensorBase
    PT_CollisionChecker=10, ///< describes \ref CollisionCheckerBase
    PT_Trajectory=11, ///< describes \ref TrajectoryBase
    PT_Viewer=12, ///< describes \ref ViewerBase
    PT_SpaceSampler=13, ///< describes \ref SamplerBase
    PT_NumberOfInterfaces=13 ///< number of interfaces, do not forget to update
};

class CollisionReport;
class InterfaceBase;
class IkSolverBase;
class TrajectoryBase;
class ControllerBase;
class PlannerBase;
class RobotBase;
class ModuleBase;
class EnvironmentBase;
class KinBody;
class SensorSystemBase;
class PhysicsEngineBase;
class SensorBase;
class CollisionCheckerBase;
class ViewerBase;
class SpaceSamplerBase;
class IkParameterization;
class ConfigurationSpecification;

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
typedef boost::shared_ptr<ModuleBase> ModuleBasePtr;
typedef boost::shared_ptr<ModuleBase const> ModuleBaseConstPtr;
typedef boost::weak_ptr<ModuleBase> ModuleBaseWeakPtr;
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
typedef boost::shared_ptr<SpaceSamplerBase> SpaceSamplerBasePtr;
typedef boost::shared_ptr<SpaceSamplerBase const> SpaceSamplerBaseConstPtr;
typedef boost::weak_ptr<SpaceSamplerBase> SpaceSamplerBaseWeakPtr;
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
class OPENRAVE_API XMLReadable
{
public:
    XMLReadable(const std::string& xmlid) : __xmlid(xmlid) {
    }
    virtual ~XMLReadable() {
    }
    virtual const std::string& GetXMLId() const {
        return __xmlid;
    }
private:
    std::string __xmlid;
};

typedef boost::shared_ptr<XMLReadable> XMLReadablePtr;
typedef boost::shared_ptr<XMLReadable const> XMLReadableConstPtr;
typedef std::list<std::pair<std::string,std::string> > AttributesList;
/// \deprecated (11/02/18)
typedef AttributesList XMLAttributesList RAVE_DEPRECATED;

/// base class for all xml readers. XMLReaders are used to process data from
/// xml files. Custom readers can be registered through EnvironmentBase.
/// By default it can record all data that is encountered inside the xml reader
class OPENRAVE_API BaseXMLReader : public boost::enable_shared_from_this<BaseXMLReader>
{
public:
    enum ProcessElement
    {
        PE_Pass=0,     ///< current tag was not supported, so pass onto another class
        PE_Support=1,     ///< current tag will be processed by this class
        PE_Ignore=2,     ///< current tag and all its children should be ignored
    };
    BaseXMLReader() {
    }
    virtual ~BaseXMLReader() {
    }

    /// a readable interface that stores the information processsed for the current tag
    /// This pointer is used to the InterfaceBase class registered readers
    virtual XMLReadablePtr GetReadable() {
        return XMLReadablePtr();
    }

    /// Gets called in the beginning of each "<type>" expression. In this case, name is "type"
    /// \param name of the tag, will be always lower case
    /// \param atts string of attributes where the first std::string is the attribute name and second is the value
    /// \return true if tag is accepted and this class will process it, otherwise false
    virtual ProcessElement startElement(const std::string& name, const AttributesList& atts) = 0;

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

typedef boost::function<BaseXMLReaderPtr(InterfaceBasePtr, const AttributesList&)> CreateXMLReaderFn;

/// reads until the tag ends
class OPENRAVE_API DummyXMLReader : public BaseXMLReader
{
public:
    DummyXMLReader(const std::string& pfieldname, const std::string& pparentname, boost::shared_ptr<std::ostream> osrecord = boost::shared_ptr<std::ostream>());
    virtual ProcessElement startElement(const std::string& name, const AttributesList& atts);
    virtual bool endElement(const std::string& name);
    virtual void characters(const std::string& ch);
    const std::string& GetFieldName() const {
        return _fieldname;
    }
private:
    std::string _parentname;     /// XML filename
    std::string _fieldname;
    boost::shared_ptr<std::ostream> _osrecord;     ///< used to store the xml data
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

#include <openrave/geometry.h>
#include <openrave/mathextra.h>

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

/// \brief The types of inverse kinematics parameterizations supported.
///
/// The minimum degree of freedoms required is set in the upper 4 bits of each type.
/// The number of values used to represent the parameterization ( >= dof ) is the next 4 bits.
/// The lower bits contain a unique id of the type.
enum IkParameterizationType {
    IKP_None=0,
    IKP_Transform6D=0x67000001,     ///< end effector reaches desired 6D transformation
    IKP_Rotation3D=0x34000002,     ///< end effector reaches desired 3D rotation
    IKP_Translation3D=0x33000003,     ///< end effector origin reaches desired 3D translation
    IKP_Direction3D=0x23000004,     ///< direction on end effector coordinate system reaches desired direction
    IKP_Ray4D=0x46000005,     ///< ray on end effector coordinate system reaches desired global ray
    IKP_Lookat3D=0x23000006,     ///< direction on end effector coordinate system points to desired 3D position
    IKP_TranslationDirection5D=0x56000007,     ///< end effector origin and direction reaches desired 3D translation and direction. Can be thought of as Ray IK where the origin of the ray must coincide.
    IKP_TranslationXY2D=0x22000008,     ///< 2D translation along XY plane
    IKP_TranslationXYOrientation3D=0x33000009,     ///< 2D translation along XY plane and 1D rotation around Z axis. The offset of the rotation is measured starting at +X, so at +X is it 0, at +Y it is pi/2.
    IKP_TranslationLocalGlobal6D=0x3600000a,     ///< local point on end effector origin reaches desired 3D global point
    IKP_NumberOfParameterizations=10,     ///< number of parameterizations (does not count IKP_None)
};

/** \brief A configuration specification references values in the environment that then define a configuration-space which can be searched for.

    It is composed of several groups targetting values for individual bodies. It is serialized into XML. The XML syntax is as follows:

   \code
   <configuration>
     <group name="string" offset="#OFF1" dof="#D1" interpolation="string"/>
     <group name="string" offset="#OFF2" dof="#D2" interpolation="string"/>
   </configuration>
   \endcode
 */
class OPENRAVE_API ConfigurationSpecification
{
public:

    /// \brief A group referencing the values of one body in the environment
    class OPENRAVE_API Group
    {
public:
        Group() : offset(0), dof(0) {
        }

        inline bool operator==(const Group& r) const {
            return offset==r.offset && dof==r.dof && name==r.name && interpolation==r.interpolation;
        }
        inline bool operator!=(const Group& r) const {
            return offset!=r.offset || dof!=r.dof || name!=r.name || interpolation!=r.interpolation;
        }

        /// \brief For each data point, the number of values to offset before data for this group starts.
        int offset;
        /// \brief The number of values in this group.
        int dof;
        /** \brief semantic information on what part of the environment the group refers to.

            Can be composed of multiple workds; the first word is the group type, and the words following narrow the specifics. Common types are:

            - \b joint_values - The joint values of a kinbody/robot. The joint names with the name of the body can follow.
            - \b joint_velocities - The joint velocities (1/second) of a kinbody/robot. The name of the body with the joint names can follow.
            - \b joint_accelerations - The joint accelerations (1/second^2) of a kinbody/robot. The name of the body with the joint names can follow.
            - \b joint_torques - The joint torques (Newton meter) of a kinbody/robot. The name of the body with the joint names can follow.
            - \b affine_transform - An affine transformation [quaternion, translation]. The name of the body with selected affine dofs (see \ref DOFAffine) can follow.
            - \b affine_velocities - The velocity (1/second) of the affine transformation [rotation axis, translation velocity], the name of the body can follow.
            - \b affine_accelerations - The velocity (1/second^2) of the affine transformation [rotation axis, translation velocity], the name of the body can follow.
            - \b ikparam_values - The values of an IkParmeterization. The ikparam type is stored as the second value in name
            - \b ikparam_velocities - velociti of an IkParmeterization. The ikparam type is stored as the second value in name
         */
        std::string name;
        /** \brief Describes how the data should be interpolated. Common methods are:

            - \b previous - the previous waypoint's value is always chosen
            - \b next - the next waypoint's value is always chosen
            - \b linear - linear interpolation (default)
            - \b quadratic - position is piecewise-quadratic, velocity is piecewise-linear, acceleration is one of -amax, 0, or amax
            - \b cubic - 3 degree polynomial
            - \b quadric - 4 degree polynomial
            - \b quintic - 5 degree polynomial
         */
        std::string interpolation;
    };

    class Reader : public BaseXMLReader
    {
public:
        Reader(ConfigurationSpecification& spec);
        virtual ProcessElement startElement(const std::string& name, const AttributesList& atts);
        virtual bool endElement(const std::string& name);
        virtual void characters(const std::string& ch);
protected:
        ConfigurationSpecification& _spec;
        std::stringstream _ss;
    };

    virtual ~ConfigurationSpecification() {
    }

    /// \brief return the dimension of the configuraiton space (degrees of freedom)
    virtual int GetDOF() const;

    /// \brief check if the groups form a continguous space
    virtual bool IsValid() const;

    virtual bool operator==(const ConfigurationSpecification& r) const;
    virtual bool operator!=(const ConfigurationSpecification& r) const;

    /// \brief finds the most compatible group to the given group
    ///
    /// \param g the group to query, only the Group::name and Group::dof values are used
    /// \param exactmatch if true, will only return groups whose name exactly matches with g.name
    /// \return an iterator part of _vgroups that represents the most compatible group. If no group is found, will return _vgroups.end()
    virtual std::vector<Group>::const_iterator FindCompatibleGroup(const Group& g, bool exactmatch=false) const;

    /// \brief Return the most compatible group that represents the time-derivative data of the group.
    ///
    /// For example given a 'joint_values' group, this will return the closest 'joint_velocities' group.
    /// \param g the group to query, only the Group::name and Group::dof values are used
    /// \param exactmatch if true, will only return groups whose name exactly matches with g.name
    /// \return an iterator part of _vgroups that represents the most compatible group. If no group is found, will return _vgroups.end()
    virtual std::vector<Group>::const_iterator FindTimeDerivativeGroup(const Group& g, bool exactmatch=false) const;

    /** \brief adds a velocity group for every position group.

        If velocities groups already exist, they are checked for and/or modified. Note that the configuration space
        might be re-ordered as a result of this function call.
        \param adddeltatime If true will add the 'deltatime' tag, which is necessary for trajectory sampling
     */
    virtual void AddVelocityGroups(bool adddeltatime);

    /// \brief converts all the groups to the corresponding velocity groups and returns the specification
    ///
    /// The velocity configuration space will have a one-to-one correspondence with the
    virtual ConfigurationSpecification ConvertToVelocitySpecification() const;

    /// \brief returns a new specification of just particular time-derivative groups.
    ///
    /// \param timederivative the time derivative to query groups from. 0 is positions/joint values, 1 is velocities, 2 is accelerations, etc
    virtual ConfigurationSpecification GetTimeDerivativeSpecification(int timederivative) const;

    /** \brief set the offsets of each group in order to get a contiguous configuration space
     */
    virtual void ResetGroupOffsets();

    /// \brief adds the deltatime tag to the end if one doesn't exist and returns the index into the configuration space
    virtual int AddDeltaTime();

    /** \brief extracts an affine transform given the start of a configuration space point

        Looks for 'affine_transform' groups. If pbody is not initialized, will choose the first affine_transform found.
        \param[inout] t the transform holding the default values, which will be overwritten with the new values.
        \param[in] itdata data in the format of this configuration specification.
        \return true if at least one group was found for extracting
     */
    virtual bool ExtractTransform(Transform& t, std::vector<dReal>::const_iterator itdata, KinBodyConstPtr pbody) const;

    /** \brief extracts an ikparameterization given the start of a configuration space point

        Looks for 'ikparam' groups.
        \param[inout] ikparam filled with ikparameterization (if found)
        \param[in] itdata data in the format of this configuration specification
        \return true if at least one group was found for extracting
     */
    virtual bool ExtractIkParameterization(IkParameterization& ikparam, std::vector<dReal>::const_iterator itdata, int timederivative=0) const;

    /** \brief extracts the affine values

        Looks for 'affine_X' groups. If pbody is not initialized, will choose the first affine_X found.
        \param[inout] itvalues iterator to vector that holds the default values and will be overwritten with the new values. must be initialized
        \param[in] itdata data in the format of this configuration specification.
        \param[in] affinedofs the format of the affine dofs requested
        \param[in] timederivative the time derivative of the data to extract
        \return true if at least one group was found for extracting
     */
    virtual bool ExtractAffineValues(std::vector<dReal>::iterator itvalues, std::vector<dReal>::const_iterator itdata, KinBodyConstPtr pbody, int affinedofs, int timederivative=0) const;

    /** \brief extracts a body's joint values given the start of a configuration space point

        Looks for 'joint_X' groups. If pbody is not initialized, will choose the first joint_X found.
        \param[inout] itvalues iterator to vector that holds the default values and will be overwritten with the new values. must be initialized
        \param[in] itdata data in the format of this configuration specification.
        \param[in] indices the set of DOF indices of the body to extract and write into itvalues.
        \param[in] timederivative the time derivative of the data to extract
        \return true if at least one group was found for extracting
     */
    virtual bool ExtractJointValues(std::vector<dReal>::iterator itvalues, std::vector<dReal>::const_iterator itdata, KinBodyConstPtr pbody, const std::vector<int>& indices, int timederivative=0) const;

    /// \brief extracts the delta time from the configuration if one exists
    virtual bool ExtractDeltaTime(dReal& deltatime, std::vector<dReal>::const_iterator itdata) const;

    /** \brief inserts a set of joint values into a configuration space point

        Looks for 'joint_X' groups. If pbody is not initialized, will use the first joint_X found.
        \param[inout] itdata data in the format of this configuration specification.
        \param[in] itvalues iterator to joint values to write
        \param[in] indices the set of DOF indices that itvalues represents.
        \param[in] timederivative the time derivative of the data to insert
        \return true if at least one group was found for inserting
     */
    virtual bool InsertJointValues(std::vector<dReal>::iterator itdata, std::vector<dReal>::const_iterator itvalues, KinBodyConstPtr pbody, const std::vector<int>& indices, int timederivative=0) const;

    /** \brief sets the deltatime field of the data if one exists

        \param[inout] itdata data in the format of this configuration specification.
        \param[in] deltatime the delta time of the time stamp (from previous point)
        \return true if at least one group was found for inserting
     */
    virtual bool InsertDeltaTime(std::vector<dReal>::iterator itdata, dReal deltatime);

    /** \brief given two compatible groups, convers data represented in the source group to data represented in the target group

        \param ittargetdata iterator pointing to start of target group data that should be overwritten
        \param targetstride the number of elements that to go from the next target point. Necessary if numpoints > 1.
        \param gtarget the target configuration group
        \param itsourcedata iterator pointing to start of source group data that should be read
        \param sourcestride the number of elements that to go from the next source point. Necessary if numpoints > 1.
        \param gsource the source configuration group
        \param numpoints the number of points to convert. The target and source strides are gtarget.dof and gsource.dof
        \param penv [optional] The environment which might be needed to fill in unknown data. Assumes environment is locked.
        \throw openrave_exception throw f groups are incompatible
     */
    static void ConvertGroupData(std::vector<dReal>::iterator ittargetdata, size_t targetstride, const Group& gtarget, std::vector<dReal>::const_iterator itsourcedata, size_t sourcestride, const Group& gsource, size_t numpoints, EnvironmentBaseConstPtr penv);

    /** \brief Converts from one specification to another.

        \param ittargetdata iterator pointing to start of target group data that should be overwritten
        \param targetspec the target configuration specification
        \param itsourcedata iterator pointing to start of source group data that should be read
        \param sourcespec the source configuration specification
        \param numpoints the number of points to convert. The target and source strides are gtarget.dof and gsource.dof
        \param penv [optional] The environment which might be needed to fill in unknown data. Assumes environment is locked.
        \param filluninitialized If there exists target groups that cannot be initialized, then will set default values to them.
     */
    static void ConvertData(std::vector<dReal>::iterator ittargetdata, const ConfigurationSpecification& targetspec, std::vector<dReal>::const_iterator itsourcedata, const ConfigurationSpecification& sourcespec, size_t numpoints, EnvironmentBaseConstPtr penv, bool filluninitialized = true);

    std::vector<Group> _vgroups;
};

OPENRAVE_API std::ostream& operator<<(std::ostream& O, const ConfigurationSpecification &spec);
OPENRAVE_API std::istream& operator>>(std::istream& I, ConfigurationSpecification& spec);

typedef boost::shared_ptr<ConfigurationSpecification> ConfigurationSpecificationPtr;
typedef boost::shared_ptr<ConfigurationSpecification const> ConfigurationSpecificationConstPtr;

/** \brief Parameterization of basic primitives for querying inverse-kinematics solutions.

    Holds the parameterization of a geometric primitive useful for autonomous manipulation scenarios like:
    6D pose, 3D translation, 3D rotation, 3D look at direction, and ray look at direction.
 */
class OPENRAVE_API IkParameterization
{
public:
    /// \deprecated (11/10/12)
    typedef IkParameterizationType Type RAVE_DEPRECATED;
    static const IkParameterizationType Type_None RAVE_DEPRECATED = IKP_None;
    static const IkParameterizationType Type_Transform6D RAVE_DEPRECATED = IKP_Transform6D;
    static const IkParameterizationType Type_Rotation3D RAVE_DEPRECATED =IKP_Rotation3D;
    static const IkParameterizationType Type_Translation3D RAVE_DEPRECATED =IKP_Translation3D;
    static const IkParameterizationType Type_Direction3D RAVE_DEPRECATED = IKP_Direction3D;
    static const IkParameterizationType Type_Ray4D RAVE_DEPRECATED = IKP_Ray4D;
    static const IkParameterizationType Type_Lookat3D RAVE_DEPRECATED = IKP_Lookat3D;
    static const IkParameterizationType Type_TranslationDirection5D RAVE_DEPRECATED = IKP_TranslationDirection5D;
    static const IkParameterizationType Type_TranslationXY2D RAVE_DEPRECATED = IKP_TranslationXY2D;
    static const IkParameterizationType Type_TranslationXYOrientation3D RAVE_DEPRECATED = IKP_TranslationXYOrientation3D;
    static const IkParameterizationType Type_TranslationLocalGlobal6D RAVE_DEPRECATED = IKP_TranslationLocalGlobal6D;
    static const IkParameterizationType Type_NumberOfParameterizations RAVE_DEPRECATED = IKP_NumberOfParameterizations;

    IkParameterization() : _type(IKP_None) {
    }
    /// \brief sets a 6D transform parameterization
    IkParameterization(const Transform &t) {
        SetTransform6D(t);
    }
    /// \brief sets a ray parameterization
    IkParameterization(const RAY &r) {
        SetRay4D(r);
    }
    /// \brief set a custom parameterization using a transform as the source of the data. Not all types are supported with this method.
    IkParameterization(const Transform &t, IkParameterizationType type) {
        _type=type;
        switch(_type) {
        case IKP_Transform6D: SetTransform6D(t); break;
        case IKP_Rotation3D: SetRotation3D(t.rot); break;
        case IKP_Translation3D: SetTranslation3D(t.trans); break;
        case IKP_Lookat3D: SetLookat3D(t.trans); break;
        default:
            throw openrave_exception(str(boost::format("IkParameterization constructor does not support type 0x%x")%_type));
        }
    }

    inline IkParameterizationType GetType() const {
        return _type;
    }
    inline const std::string& GetName() const;

    /// \brief Returns the minimum degree of freedoms required for the IK type.
    static int GetDOF(IkParameterizationType type) {
        return (type>>28)&0xf;
    }
    /// \brief Returns the minimum degree of freedoms required for the IK type.
    inline int GetDOF() const {
        return (_type>>28)&0xf;
    }

    /// \brief Returns the number of values used to represent the parameterization ( >= dof ). The number of values serialized is this number plus 1 for the iktype.
    static int GetNumberOfValues(IkParameterizationType type) {
        return (type>>24)&0xf;
    }
    /// \brief Returns the number of values used to represent the parameterization ( >= dof ). The number of values serialized is this number plus 1 for the iktype.
    inline int GetNumberOfValues() const {
        return (_type>>24)&0xf;
    }

    inline void SetTransform6D(const Transform& t) {
        _type = IKP_Transform6D; _transform = t;
    }
    inline void SetRotation3D(const Vector& quaternion) {
        _type = IKP_Rotation3D; _transform.rot = quaternion;
    }
    inline void SetTranslation3D(const Vector& trans) {
        _type = IKP_Translation3D; _transform.trans = trans;
    }
    inline void SetDirection3D(const Vector& dir) {
        _type = IKP_Direction3D; _transform.rot = dir;
    }
    inline void SetRay4D(const RAY& ray) {
        _type = IKP_Ray4D; _transform.trans = ray.pos; _transform.rot = ray.dir;
    }
    inline void SetLookat3D(const Vector& trans) {
        _type = IKP_Lookat3D; _transform.trans = trans;
    }
    /// \brief the ray direction is not used for IK, however it is needed in order to compute the error
    inline void SetLookat3D(const RAY& ray) {
        _type = IKP_Lookat3D; _transform.trans = ray.pos; _transform.rot = ray.dir;
    }
    inline void SetTranslationDirection5D(const RAY& ray) {
        _type = IKP_TranslationDirection5D; _transform.trans = ray.pos; _transform.rot = ray.dir;
    }
    inline void SetTranslationXY2D(const Vector& trans) {
        _type = IKP_TranslationXY2D; _transform.trans.x = trans.x; _transform.trans.y = trans.y; _transform.trans.z = 0; _transform.trans.w = 0;
    }
    inline void SetTranslationXYOrientation3D(const Vector& trans) {
        _type = IKP_TranslationXYOrientation3D; _transform.trans.x = trans.x; _transform.trans.y = trans.y; _transform.trans.z = trans.z; _transform.trans.w = 0;
    }
    inline void SetTranslationLocalGlobal6D(const Vector& localtrans, const Vector& trans) {
        _type = IKP_TranslationLocalGlobal6D; _transform.rot.x = localtrans.x; _transform.rot.y = localtrans.y; _transform.rot.z = localtrans.z; _transform.rot.w = 0; _transform.trans.x = trans.x; _transform.trans.y = trans.y; _transform.trans.z = trans.z; _transform.trans.w = 0;
    }

    inline const Transform& GetTransform6D() const {
        return _transform;
    }
    inline const Vector& GetRotation3D() const {
        return _transform.rot;
    }
    inline const Vector& GetTranslation3D() const {
        return _transform.trans;
    }
    inline const Vector& GetDirection3D() const {
        return _transform.rot;
    }
    inline const RAY GetRay4D() const {
        return RAY(_transform.trans,_transform.rot);
    }
    inline const Vector& GetLookat3D() const {
        return _transform.trans;
    }
    inline const Vector& GetLookat3DDirection() const {
        return _transform.rot;
    }
    inline const RAY GetTranslationDirection5D() const {
        return RAY(_transform.trans,_transform.rot);
    }
    inline const Vector& GetTranslationXY2D() const {
        return _transform.trans;
    }
    inline const Vector& GetTranslationXYOrientation3D() const {
        return _transform.trans;
    }
    inline std::pair<Vector,Vector> GetTranslationLocalGlobal6D() const {
        return std::make_pair(_transform.rot,_transform.trans);
    }

    /// \deprecated (11/02/15)
    //@{
    inline void SetTransform(const Transform& t) RAVE_DEPRECATED {
        SetTransform6D(t);
    }
    inline void SetRotation(const Vector& quaternion) RAVE_DEPRECATED {
        SetRotation3D(quaternion);
    }
    inline void SetTranslation(const Vector& trans) RAVE_DEPRECATED {
        SetTranslation3D(trans);
    }
    inline void SetDirection(const Vector& dir) RAVE_DEPRECATED {
        SetDirection3D(dir);
    }
    inline void SetRay(const RAY& ray) RAVE_DEPRECATED {
        SetRay4D(ray);
    }
    inline void SetLookat(const Vector& trans) RAVE_DEPRECATED {
        SetLookat3D(trans);
    }
    inline void SetTranslationDirection(const RAY& ray) RAVE_DEPRECATED {
        SetTranslationDirection5D(ray);
    }
    inline const Transform& GetTransform() const RAVE_DEPRECATED {
        return _transform;
    }
    inline const Vector& GetRotation() const RAVE_DEPRECATED {
        return _transform.rot;
    }
    inline const Vector& GetTranslation() const RAVE_DEPRECATED {
        return _transform.trans;
    }
    inline const Vector& GetDirection() const RAVE_DEPRECATED {
        return _transform.rot;
    }
    inline const Vector& GetLookat() const RAVE_DEPRECATED {
        return _transform.trans;
    }
    inline const RAY GetRay() const RAVE_DEPRECATED {
        return RAY(_transform.trans,_transform.rot);
    }
    inline const RAY GetTranslationDirection() const RAVE_DEPRECATED {
        return RAY(_transform.trans,_transform.rot);
    }
    //@}


    /// \brief Computes the distance squared between two IK parmaeterizations.
    inline dReal ComputeDistanceSqr(const IkParameterization& ikparam) const
    {
        const dReal anglemult = 0.4;     // this is a hack that should be removed....
        BOOST_ASSERT(_type==ikparam.GetType());
        switch(_type) {
        case IKP_Transform6D: {
            Transform t0 = GetTransform6D(), t1 = ikparam.GetTransform6D();
            dReal fcos = RaveFabs(t0.rot.dot(t1.rot));
            dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
            return (t0.trans-t1.trans).lengthsqr3() + anglemult*facos*facos;
        }
        case IKP_Rotation3D: {
            dReal fcos = RaveFabs(GetRotation3D().dot(ikparam.GetRotation3D()));
            dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
            return facos*facos;
        }
        case IKP_Translation3D:
            return (GetTranslation3D()-ikparam.GetTranslation3D()).lengthsqr3();
        case IKP_Direction3D: {
            dReal fcos = GetDirection3D().dot(ikparam.GetDirection3D());
            dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
            return facos*facos;
        }
        case IKP_Ray4D: {
            Vector pos0 = GetRay4D().pos - GetRay4D().dir*GetRay4D().dir.dot(GetRay4D().pos);
            Vector pos1 = ikparam.GetRay4D().pos - ikparam.GetRay4D().dir*ikparam.GetRay4D().dir.dot(ikparam.GetRay4D().pos);
            dReal fcos = GetRay4D().dir.dot(ikparam.GetRay4D().dir);
            dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
            return (pos0-pos1).lengthsqr3() + anglemult*facos*facos;
        }
        case IKP_Lookat3D: {
            Vector v = GetLookat3D()-ikparam.GetLookat3D();
            dReal s = v.dot3(ikparam.GetLookat3DDirection());
            if( s >= -1 ) {     // ikparam's lookat is always 1 beyond the origin, this is just the convention for testing...
                v -= s*ikparam.GetLookat3DDirection();
            }
            return v.lengthsqr3();
        }
        case IKP_TranslationDirection5D: {
            dReal fcos = GetTranslationDirection5D().dir.dot(ikparam.GetTranslationDirection5D().dir);
            dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
            return (GetTranslationDirection5D().pos-ikparam.GetTranslationDirection5D().pos).lengthsqr3() + anglemult*facos*facos;
        }
        case IKP_TranslationXY2D: {
            return (GetTranslationXY2D()-ikparam.GetTranslationXY2D()).lengthsqr2();
        }
        case IKP_TranslationXYOrientation3D: {
            Vector v0 = GetTranslationXYOrientation3D();
            Vector v1 = ikparam.GetTranslationXYOrientation3D();
            dReal anglediff = v0.z-v1.z;
            if (anglediff < dReal(-PI)) {
                anglediff += dReal(2*PI);
                while (anglediff < dReal(-PI))
                    anglediff += dReal(2*PI);
            }
            else if (anglediff > dReal(PI)) {
                anglediff -= dReal(2*PI);
                while (anglediff > dReal(PI))
                    anglediff -= dReal(2*PI);
            }
            return (v0-v1).lengthsqr2() + anglemult*anglediff*anglediff;
        }
        case IKP_TranslationLocalGlobal6D: {
            std::pair<Vector,Vector> p0 = GetTranslationLocalGlobal6D(), p1 = ikparam.GetTranslationLocalGlobal6D();
            return (p0.first-p1.first).lengthsqr3() + (p0.second-p1.second).lengthsqr3();
        }
        default:
            BOOST_ASSERT(0);
        }
        return 1e30;
    }

    /// \brief fills the iterator with the serialized values of the ikparameterization.
    ///
    /// the container the iterator points to needs to have \ref GetNumberOfValues() available.
    inline void GetValues(std::vector<dReal>::iterator itvalues) const
    {
        switch(_type) {
        case IKP_Transform6D:
            *itvalues++ = _transform.rot.x;
            *itvalues++ = _transform.rot.y;
            *itvalues++ = _transform.rot.z;
            *itvalues++ = _transform.rot.w;
            *itvalues++ = _transform.trans.x;
            *itvalues++ = _transform.trans.y;
            *itvalues++ = _transform.trans.z;
            break;
        case IKP_Rotation3D:
            *itvalues++ = _transform.rot.x;
            *itvalues++ = _transform.rot.y;
            *itvalues++ = _transform.rot.z;
            *itvalues++ = _transform.rot.w;
            break;
        case IKP_Translation3D:
            *itvalues++ = _transform.trans.x;
            *itvalues++ = _transform.trans.y;
            *itvalues++ = _transform.trans.z;
            break;
        case IKP_Direction3D:
            *itvalues++ = _transform.rot.x;
            *itvalues++ = _transform.rot.y;
            *itvalues++ = _transform.rot.z;
            break;
        case IKP_Ray4D:
            *itvalues++ = _transform.rot.x;
            *itvalues++ = _transform.rot.y;
            *itvalues++ = _transform.rot.z;
            *itvalues++ = _transform.trans.x;
            *itvalues++ = _transform.trans.y;
            *itvalues++ = _transform.trans.z;
            break;
        case IKP_Lookat3D:
            *itvalues++ = _transform.trans.x;
            *itvalues++ = _transform.trans.y;
            *itvalues++ = _transform.trans.z;
            break;
        case IKP_TranslationDirection5D:
            *itvalues++ = _transform.rot.x;
            *itvalues++ = _transform.rot.y;
            *itvalues++ = _transform.rot.z;
            *itvalues++ = _transform.trans.x;
            *itvalues++ = _transform.trans.y;
            *itvalues++ = _transform.trans.z;
            break;
        case IKP_TranslationXY2D:
            *itvalues++ = _transform.trans.x;
            *itvalues++ = _transform.trans.y;
            break;
        case IKP_TranslationXYOrientation3D:
            *itvalues++ = _transform.trans.x;
            *itvalues++ = _transform.trans.y;
            *itvalues++ = _transform.trans.z;
            break;
        case IKP_TranslationLocalGlobal6D:
            *itvalues++ = _transform.rot.x;
            *itvalues++ = _transform.rot.y;
            *itvalues++ = _transform.rot.z;
            *itvalues++ = _transform.trans.x;
            *itvalues++ = _transform.trans.y;
            *itvalues++ = _transform.trans.z;
            break;
        default:
            throw OPENRAVE_EXCEPTION_FORMAT("does not support parameterization 0x%x", _type,ORE_InvalidArguments);
        }
    }

    inline void Set(std::vector<dReal>::const_iterator itvalues, IkParameterizationType iktype)
    {
        _type = iktype;
        switch(_type) {
        case IKP_Transform6D:
            _transform.rot.x = *itvalues++;
            _transform.rot.y = *itvalues++;
            _transform.rot.z = *itvalues++;
            _transform.rot.w = *itvalues++;
            _transform.trans.x = *itvalues++;
            _transform.trans.y = *itvalues++;
            _transform.trans.z = *itvalues++;
            break;
        case IKP_Rotation3D:
            _transform.rot.x = *itvalues++;
            _transform.rot.y = *itvalues++;
            _transform.rot.z = *itvalues++;
            _transform.rot.w = *itvalues++;
            break;
        case IKP_Translation3D: {
            _transform.trans.x = *itvalues++;
            _transform.trans.y = *itvalues++;
            _transform.trans.z = *itvalues++;
            break;
        }
        case IKP_Direction3D: {
            _transform.rot.x = *itvalues++;
            _transform.rot.y = *itvalues++;
            _transform.rot.z = *itvalues++;
            break;
        }
        case IKP_Ray4D: {
            _transform.rot.x = *itvalues++;
            _transform.rot.y = *itvalues++;
            _transform.rot.z = *itvalues++;
            _transform.trans.x = *itvalues++;
            _transform.trans.y = *itvalues++;
            _transform.trans.z = *itvalues++;
            break;
        }
        case IKP_Lookat3D: {
            _transform.trans.x = *itvalues++;
            _transform.trans.y = *itvalues++;
            _transform.trans.z = *itvalues++;
            break;
        }
        case IKP_TranslationDirection5D:
            _transform.rot.x = *itvalues++;
            _transform.rot.y = *itvalues++;
            _transform.rot.z = *itvalues++;
            _transform.trans.x = *itvalues++;
            _transform.trans.y = *itvalues++;
            _transform.trans.z = *itvalues++;
            break;
        case IKP_TranslationXY2D: {
            _transform.trans.x = *itvalues++;
            _transform.trans.y = *itvalues++;
            break;
        }
        case IKP_TranslationXYOrientation3D: {
            _transform.trans.x = *itvalues++;
            _transform.trans.y = *itvalues++;
            _transform.trans.z = *itvalues++;
            break;
        }
        case IKP_TranslationLocalGlobal6D: {
            _transform.rot.x = *itvalues++;
            _transform.rot.y = *itvalues++;
            _transform.rot.z = *itvalues++;
            _transform.trans.x = *itvalues++;
            _transform.trans.y = *itvalues++;
            _transform.trans.z = *itvalues++;
            break;
        }
        default:
            throw OPENRAVE_EXCEPTION_FORMAT("does not support parameterization 0x%x", _type,ORE_InvalidArguments);
        }
    }

    static ConfigurationSpecification GetConfigurationSpecification(IkParameterizationType iktype);
    inline ConfigurationSpecification GetConfigurationSpecification() const
    {
        return GetConfigurationSpecification(GetType());
    }

protected:
    Transform _transform;
    IkParameterizationType _type;

    friend IkParameterization operator* (const Transform &t, const IkParameterization &ikparam);
    friend OPENRAVE_API std::ostream& operator<<(std::ostream& O, const IkParameterization &ikparam);
    friend OPENRAVE_API std::istream& operator>>(std::istream& I, IkParameterization& ikparam);
};

inline IkParameterization operator* (const Transform &t, const IkParameterization &ikparam)
{
    IkParameterization local;
    switch(ikparam.GetType()) {
    case IKP_Transform6D:
        local.SetTransform6D(t * ikparam.GetTransform6D());
        break;
    case IKP_Rotation3D:
        local.SetRotation3D(quatMultiply(quatInverse(t.rot),ikparam.GetRotation3D()));
        break;
    case IKP_Translation3D:
        local.SetTranslation3D(t*ikparam.GetTranslation3D());
        break;
    case IKP_Direction3D:
        local.SetDirection3D(t.rotate(ikparam.GetDirection3D()));
        break;
    case IKP_Ray4D:
        local.SetRay4D(RAY(t*ikparam.GetRay4D().pos,t.rotate(ikparam.GetRay4D().dir)));
        break;
    case IKP_Lookat3D:
        local.SetLookat3D(RAY(t*ikparam.GetLookat3D(),t.rotate(ikparam.GetLookat3DDirection())));
        break;
    case IKP_TranslationDirection5D:
        local.SetTranslationDirection5D(RAY(t*ikparam.GetTranslationDirection5D().pos,t.rotate(ikparam.GetTranslationDirection5D().dir)));
        break;
    case IKP_TranslationXY2D:
        local.SetTranslationXY2D(t*ikparam.GetTranslationXY2D());
        break;
    case IKP_TranslationXYOrientation3D: {
        Vector v = ikparam.GetTranslationXYOrientation3D();
        Vector voldtrans(v.x,v.y,0);
        Vector vnewtrans = t*voldtrans;
        dReal zangle = -normalizeAxisRotation(Vector(0,0,1),t.rot).first;
        local.SetTranslationXYOrientation3D(Vector(vnewtrans.y,vnewtrans.y,v.z+zangle));
        break;
    }
    case IKP_TranslationLocalGlobal6D:
        local.SetTranslationLocalGlobal6D(ikparam.GetTranslationLocalGlobal6D().first, t*ikparam.GetTranslationLocalGlobal6D().second);
        break;
    default:
        throw openrave_exception(str(boost::format("does not support parameterization %d")%ikparam.GetType()));
    }
    return local;
}

OPENRAVE_API std::ostream& operator<<(std::ostream& O, const IkParameterization &ikparam);
OPENRAVE_API std::istream& operator>>(std::istream& I, IkParameterization& ikparam);

/// \brief Selects which DOFs of the affine transformation to include in the active configuration.
enum DOFAffine
{
    DOF_NoTransform = 0,
    DOF_X = 1,     ///< can move in the x direction
    DOF_Y = 2,     ///< can move in the y direction
    DOF_Z = 4,     ///< can move in the z direction
    DOF_XYZ=DOF_X|DOF_Y|DOF_Z,     ///< moves in xyz direction

    // DOF_RotationX fields are mutually exclusive
    DOF_RotationAxis = 8,     ///< can rotate around an axis (1 dof)
    DOF_Rotation3D = 16,     ///< can rotate freely (3 dof), the parameterization is
                             ///< theta * v, where v is the rotation axis and theta is the angle about that axis
    DOF_RotationQuat = 32,     ///< can rotate freely (4 dof), parameterization is a quaternion. In order for limits to work correctly, the quaternion is in the space of _vRotationQuatLimitStart. _vRotationQuatLimitStart is always left-multiplied before setting the transform!
    DOF_RotationMask=(DOF_RotationAxis|DOF_Rotation3D|DOF_RotationQuat), ///< mask for all bits representing 3D rotations
    DOF_Transform = (DOF_XYZ|DOF_RotationQuat), ///< translate and rotate freely in 3D space
};

/** \brief Given a mask of affine dofs and a dof inside that mask, returns the index where the value could be found.

    \param affinedofs a mask of \ref DOFAffine values
    \param dof a set of values inside affinedofs, the index of the first value is returned
    \throw openrave_exception throws if dof is not present in affinedofs
 */
OPENRAVE_API int RaveGetIndexFromAffineDOF(int affinedofs, DOFAffine dof);

/** \brief Given a mask of affine dofs and an index into the array, returns the affine dof that is being referenced

    \param affinedofs a mask of \ref DOFAffine values
    \param index an index into the affine dof array
    \throw openrave_exception throws if dof if index is out of bounds
 */
OPENRAVE_API DOFAffine RaveGetAffineDOFFromIndex(int affinedofs, int index);

/// \brief Returns the degrees of freedom needed to represent all the values in the affine dof mask.
///
/// \throw openrave_exception throws if
OPENRAVE_API int RaveGetAffineDOF(int affinedofs);

/** \brief Converts the transformation matrix into the specified affine values format.

    \param[out] itvalues an iterator to the vector to write the values to. Will write exactly \ref RaveGetAffineDOF(affinedofs) values.
    \param[in] the affine transformation to convert
    \param[in] affinedofs the affine format to output values in
    \param[in] vActvAffineRotationAxis optional rotation axis if affinedofs specified \ref DOF_RotationAxis
 */
OPENRAVE_API void RaveGetAffineDOFValuesFromTransform(std::vector<dReal>::iterator itvalues, const Transform& t, int affinedofs, const Vector& vActvAffineRotationAxis=Vector(0,0,1));

/** \brief Converts affine dof values into a transform.

    Note that depending on what the dof values holds, only a part of the transform will be updated.
    \param[out] t the output transform
    \param[in] itvalues the start iterator of the affine dof values
    \param[in] affinedofs the affine dof mask
    \param[in] vActvAffineRotationAxis optional rotation axis if affinedofs specified \ref DOF_RotationAxis
 */
OPENRAVE_API void RaveGetTransformFromAffineDOFValues(Transform& t, std::vector<dReal>::const_iterator itvalues, int affinedofs, const Vector& vActvAffineRotationAxis=Vector(0,0,1));

OPENRAVE_API ConfigurationSpecification RaveGetAffineConfigurationSpecification(int affinedofs,KinBodyConstPtr pbody=KinBodyConstPtr());

}

#include <openrave/plugininfo.h>
#include <openrave/interface.h>
#include <openrave/spacesampler.h>
#include <openrave/kinbody.h>
#include <openrave/trajectory.h>
#include <openrave/module.h>
#include <openrave/collisionchecker.h>
#include <openrave/sensor.h>
#include <openrave/robot.h>
#include <openrave/iksolver.h>
#include <openrave/planner.h>
#include <openrave/controller.h>
#include <openrave/physicsengine.h>
#include <openrave/sensorsystem.h>
#include <openrave/viewer.h>
#include <openrave/environment.h>

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
    case PT_Module: return OPENRAVE_MODULE_HASH;
    case PT_InverseKinematicsSolver: return OPENRAVE_IKSOLVER_HASH;
    case PT_KinBody: return OPENRAVE_KINBODY_HASH;
    case PT_PhysicsEngine: return OPENRAVE_PHYSICSENGINE_HASH;
    case PT_Sensor: return OPENRAVE_SENSOR_HASH;
    case PT_CollisionChecker: return OPENRAVE_COLLISIONCHECKER_HASH;
    case PT_Trajectory: return OPENRAVE_TRAJECTORY_HASH;
    case PT_Viewer: return OPENRAVE_VIEWER_HASH;
    case PT_SpaceSampler: return OPENRAVE_SPACESAMPLER_HASH;
    default:
        throw openrave_exception("failed to find openrave interface type",ORE_InvalidArguments);
        return NULL;
    }
}

/// \brief Safely casts from the base interface class to an openrave interface using static_pointer_cast.
///
/// The reason why dynamic_pointer_cast cannot be used is because interfaces might be created by different plugins, and the runtime type information will be different.
template <typename T>
inline boost::shared_ptr<T> RaveInterfaceCast(InterfaceBasePtr pinterface)
{
    if( !!pinterface ) {
        if( pinterface->GetInterfaceType() == T::GetInterfaceTypeStatic() ) {
            return boost::static_pointer_cast<T>(pinterface);
        }
        // encode special cases
        if((pinterface->GetInterfaceType() == PT_Robot)&&(T::GetInterfaceTypeStatic() == PT_KinBody)) {
            return boost::static_pointer_cast<T>(pinterface);
        }
    }
    return boost::shared_ptr<T>();
}

/// \brief Safely casts from the base interface class to an openrave interface using static_pointer_cast.
///
/// The reason why dynamic_pointer_cast cannot be used is because interfaces might be created by different plugins, and the runtime type information will be different.
template <typename T>
inline boost::shared_ptr<T const> RaveInterfaceConstCast(InterfaceBaseConstPtr pinterface)
{
    if( !!pinterface ) {
        if( pinterface->GetInterfaceType() == T::GetInterfaceTypeStatic() ) {
            return boost::static_pointer_cast<T const>(pinterface);
        }
        // encode special cases
        if((pinterface->GetInterfaceType() == PT_Robot)&&(T::GetInterfaceTypeStatic() == PT_KinBody)) {
            return boost::static_pointer_cast<T const>(pinterface);
        }
    }
    return boost::shared_ptr<T>();
}

/// \brief returns a lower case string of the interface type
OPENRAVE_API const std::map<InterfaceType,std::string>& RaveGetInterfaceNamesMap();
OPENRAVE_API const std::string& RaveGetInterfaceName(InterfaceType type);

/// \brief returns a string of the ik parameterization type names (can include upper case in order to match \ref IkParameterizationType)
OPENRAVE_API const std::map<IkParameterizationType,std::string>& RaveGetIkParameterizationMap();

/// \brief Returns the openrave home directory where settings, cache, and other files are stored.
///
/// On Linux/Unix systems, this is usually $HOME/.openrave, on Windows this is $HOMEPATH/.openrave
OPENRAVE_API std::string RaveGetHomeDirectory();

/// \brief Searches for a filename in the database and returns a full path/URL to it
///
/// \param filename the relative filename in the database
/// \param bRead if true will only return a file if it exists. If false, will return the filename of the first valid database directory.
/// \return a non-empty string if a file could be found.
OPENRAVE_API std::string RaveFindDatabaseFile(const std::string& filename, bool bRead=true);

/// \brief Explicitly initializes the global OpenRAVE state (optional).
///
/// Optional function to initialize openrave plugins and logging.
/// Although environment creation will automatically make sure this function is called, users might want
/// explicit control of when this happens.
/// \param bLoadAllPlugins If true will load all the openrave plugins automatically that can be found in the OPENRAVE_PLUGINS environment path
/// \return 0 if successful, otherwise an error code
OPENRAVE_API int RaveInitialize(bool bLoadAllPlugins=true, int level = Level_Info);

/// \brief Initializes the global state from an already loaded OpenRAVE environment.
///
/// Because of shared object boundaries, it is necessary to pass the global state pointer
/// around. If using plugin.h, this function is automatically called by \ref CreateInterfaceValidated.
/// It is also called by and every InterfaceBase constructor.
/// \param[in] globalstate
OPENRAVE_API void RaveInitializeFromState(UserDataPtr globalstate);

/// \brief A pointer to the global openrave state
/// \return a managed pointer to the state.
OPENRAVE_API UserDataPtr RaveGlobalState();

/// \brief Destroys the entire OpenRAVE state and all loaded environments.
///
/// This functions should be always called before program shutdown in order to assure all
/// resources are relased appropriately.
OPENRAVE_API void RaveDestroy();

/// \brief Get all the loaded plugins and the interfaces they support.
///
/// \param plugins A list of plugins. Each entry has the plugin name and the interfaces it supports
OPENRAVE_API void RaveGetPluginInfo(std::list< std::pair<std::string, PLUGININFO> >& plugins);

/// \brief Get a list of all the loaded interfaces.
OPENRAVE_API void RaveGetLoadedInterfaces(std::map<InterfaceType, std::vector<std::string> >& interfacenames);

/// \brief Reloads all the plugins.
///
/// The interfaces currently created remain will continue using the old plugins, so this function is safe in that plugins currently loaded remain loaded until the last interface that uses them is released.
OPENRAVE_API void RaveReloadPlugins();

/// \brief Load a plugin and its interfaces.
///
/// If the plugin is already loaded, will reload it.
/// \param name the filename of the plugin to load
OPENRAVE_API bool RaveLoadPlugin(const std::string& libraryname);

/// \brief Returns true if interface can be created, otherwise false.
OPENRAVE_API bool RaveHasInterface(InterfaceType type, const std::string& interfacename);

OPENRAVE_API InterfaceBasePtr RaveCreateInterface(EnvironmentBasePtr penv, InterfaceType type,const std::string& interfacename);
OPENRAVE_API RobotBasePtr RaveCreateRobot(EnvironmentBasePtr penv, const std::string& name="");
OPENRAVE_API PlannerBasePtr RaveCreatePlanner(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API SensorSystemBasePtr RaveCreateSensorSystem(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API ControllerBasePtr RaveCreateController(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API ModuleBasePtr RaveCreateModule(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API ModuleBasePtr RaveCreateProblem(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API ModuleBasePtr RaveCreateProblemInstance(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API IkSolverBasePtr RaveCreateIkSolver(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API PhysicsEngineBasePtr RaveCreatePhysicsEngine(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API SensorBasePtr RaveCreateSensor(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API CollisionCheckerBasePtr RaveCreateCollisionChecker(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API ViewerBasePtr RaveCreateViewer(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API SpaceSamplerBasePtr RaveCreateSpaceSampler(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API KinBodyPtr RaveCreateKinBody(EnvironmentBasePtr penv, const std::string& name="");
/// \brief Return an empty trajectory instance.
OPENRAVE_API TrajectoryBasePtr RaveCreateTrajectory(EnvironmentBasePtr penv, const std::string& name="");

OPENRAVE_API TrajectoryBasePtr RaveCreateTrajectory(EnvironmentBasePtr penv, int dof) RAVE_DEPRECATED;

/** \brief Registers a function to create an interface, this allows the interface to be created by other modules.

    \param type interface type
    \param name interface name
    \param interfacehash the hash of the interface being created (use the global defines OPENRAVE_X_HASH)
    \param envhash the hash of the environment (use the global define OPENRAVE_ENVIRONMENT_HASH)
    \param createfn functions to create the interface it takes two parameters: the environment and an istream to the rest of the interface creation arguments.
    \return a handle if function is successfully registered. By destroying the handle, the interface will be automatically unregistered.
    \throw openrave_exception Will throw with ORE_InvalidInterfaceHash if hashes do not match
 */
OPENRAVE_API UserDataPtr RaveRegisterInterface(InterfaceType type, const std::string& name, const char* interfacehash, const char* envhash, const boost::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)>& createfn);

/** \brief Registers a custom xml reader for a particular interface.

    Once registered, anytime an interface is created through XML and
    the xmltag is seen, the function CreateXMLReaderFn will be called to get a reader for that tag
    \param xmltag the tag specified in xmltag is seen in the interface, the the custom reader will be created.
    \param fn CreateXMLReaderFn(pinterface,atts) - passed in the pointer to the interface where the tag was seen along with the list of attributes
    \return a pointer holding the registration, releasing the pointer will unregister the XML reader
 */
OPENRAVE_API UserDataPtr RaveRegisterXMLReader(InterfaceType type, const std::string& xmltag, const CreateXMLReaderFn& fn);

/// \brief return the environment's unique id, returns 0 if environment could not be found or not registered
OPENRAVE_API int RaveGetEnvironmentId(EnvironmentBasePtr penv);

/// \brief get the environment from its unique id
/// \param id the unique environment id returned by \ref RaveGetEnvironmentId
OPENRAVE_API EnvironmentBasePtr RaveGetEnvironment(int id);

/// \brief Return all the created OpenRAVE environments.
OPENRAVE_API void RaveGetEnvironments(std::list<EnvironmentBasePtr>& listenvironments);

/// \brief Returns the current registered reader for the interface type/xmlid
///
/// \throw openrave_exception Will throw with ORE_InvalidArguments if registered function could not be found.
OPENRAVE_API BaseXMLReaderPtr RaveCallXMLReader(InterfaceType type, const std::string& xmltag, InterfaceBasePtr pinterface, const AttributesList& atts);

//@}

/// \deprecated (11/06/03), use \ref SpaceSamplerBase
OPENRAVE_API void RaveInitRandomGeneration(uint32_t seed);
/// \deprecated (11/06/03), use \ref SpaceSamplerBase
OPENRAVE_API uint32_t RaveRandomInt();
/// \deprecated (11/06/03), use \ref SpaceSamplerBase
OPENRAVE_API float RaveRandomFloat(IntervalType interval=IT_Closed);
/// \deprecated (11/06/03), use \ref SpaceSamplerBase
OPENRAVE_API double RaveRandomDouble(IntervalType interval=IT_Closed);

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

// define inline functions
const std::string& IkParameterization::GetName() const
{
    std::map<IkParameterizationType,std::string>::const_iterator it = RaveGetIkParameterizationMap().find(_type);
    if( it != RaveGetIkParameterizationMap().end() ) {
        return it->second;
    }
    throw openrave_exception(str(boost::format("IkParameterization iktype 0x%x not supported")));
}

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
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::UserData)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::ModuleBase)
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
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::SpaceSamplerBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::GraphHandle)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::IkParameterization)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::ConfigurationSpecification)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::ConfigurationSpecification::Group)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::ConfigurationSpecification::Reader)
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
