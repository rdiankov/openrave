// -*- coding: utf-8 -*-
// Copyright (C) 2006-2016 Rosen Diankov <rosen.diankov@gmail.com>
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
/** \file logging.h
    \brief Defines logging related utilities. This file is automatically included by openrave.h.
 */
#ifndef OPENRAVE_LOGGING_H
#define OPENRAVE_LOGGING_H

#if OPENRAVE_LOG4CXX
#include <log4cxx/logger.h>
#endif

#include <vector>

namespace OpenRAVE {

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

#define RAVEPRINTHEADER(LEVEL) OpenRAVE::RavePrintfA ## LEVEL("[%s:%d %s] ", OpenRAVE::RaveGetSourceFilename(__FILE__), __LINE__,  __FUNCTION__)

// different logging levels. The higher the suffix number, the less important the information is.
// 0 log level logs all the time. OpenRAVE starts up with a log level of 0.
#define RAVELOG_LEVELW(LEVEL,level,...) do { if (int(OpenRAVE::RaveGetDebugLevel()&OpenRAVE::Level_OutputMask)>=int(level)) { RAVEPRINTHEADER(LEVEL); OpenRAVE::RavePrintfW ## LEVEL(__VA_ARGS__); } } while (0)
#define RAVELOG_LEVELA(LEVEL,level,...) do { if (int(OpenRAVE::RaveGetDebugLevel()&OpenRAVE::Level_OutputMask)>=int(level)) { RAVEPRINTHEADER(LEVEL); OpenRAVE::RavePrintfA ## LEVEL(__VA_ARGS__); } } while (0)


#if OPENRAVE_LOG4CXX

/// \brief Get the global log4cxx logger.
OPENRAVE_API log4cxx::LoggerPtr RaveGetLogger();

/// \brief Get the verbose log4cxx level. The only difference between this and log4cxx::Level::getTrace() is the text VERBOSE.
OPENRAVE_API log4cxx::LevelPtr RaveGetVerboseLogLevel();

#ifdef LOG4CXX_LOCATION
#undef LOG4CXX_LOCATION
#endif
#define LOG4CXX_LOCATION ::log4cxx::spi::LocationInfo(OpenRAVE::RaveGetSourceFilename(__FILE__), __LOG4CXX_FUNC__, __LINE__)

#define OPENRAVE_LOG4CXX_FATALLEVEL(logger, message, location) {if (!!logger && logger->isFatalEnabled()) { logger->forcedLog(::log4cxx::Level::getFatal(), message, location); }}
#define OPENRAVE_LOG4CXX_ERRORLEVEL(logger, message, location) {if (!!logger && logger->isErrorEnabled()) { logger->forcedLog(::log4cxx::Level::getError(), message, location); }}
#define OPENRAVE_LOG4CXX_WARNLEVEL(logger, message, location) {if (!!logger && logger->isWarnEnabled()) { logger->forcedLog(::log4cxx::Level::getWarn(), message, location); }}
#define OPENRAVE_LOG4CXX_INFOLEVEL(logger, message, location) {if (!!logger && logger->isInfoEnabled()) { logger->forcedLog(::log4cxx::Level::getInfo(), message, location); }}
#define OPENRAVE_LOG4CXX_DEBUGLEVEL(logger, message, location) {if (!!logger && logger->isDebugEnabled()) { logger->forcedLog(::log4cxx::Level::getDebug(), message, location); }}
#define OPENRAVE_LOG4CXX_VERBOSELEVEL(logger, message, location) {if (!!logger && logger->isTraceEnabled()) { logger->forcedLog(OpenRAVE::RaveGetVerboseLogLevel(), message, location); }}

#define DefineRavePrintfW(LEVEL) \
    inline int RavePrintfW ## LEVEL(const log4cxx::LoggerPtr& logger, const log4cxx::spi::LocationInfo& location, const wchar_t *wfmt, ...) \
    { \
        va_list list; \
        wchar_t wbuf[512]; /* wide char buffer to hold vswprintf result */ \
        wchar_t* ws = &wbuf[0]; \
        wchar_t* wsallocated = NULL; /* allocated wide char buffer */ \
        int wslen = sizeof(wbuf)/sizeof(wchar_t); /* wide char buffer length (character count) */ \
        int wr = -1; \
        \
        va_start(list, wfmt); \
        for (;;) { \
            wr = vswprintf(ws, wslen, wfmt, list); \
            if (wr >= 0) { \
                break; \
            } \
            if (wslen >= 16384) { \
                wr = -1; \
                break; \
            } \
            /* vswprintf does not tell us how much space is needed, so we need to grow until it is satisfied */ \
            wslen *= 2; \
            wsallocated = (wchar_t*)realloc(wsallocated, wslen*sizeof(wchar_t)); \
            ws = wsallocated; \
        } \
        if (wr >= 0) { \
            /* get rid of the trailing \n if presnet */ \
            if (wr > 0 && ws[wr-1] == L'\n') { \
                ws[wr-1] = '\0'; \
            } \
            if (!!logger) { \
                OPENRAVE_LOG4CXX ## LEVEL(logger, ws, location); \
            } else { \
                wprintf(L"%ls\n", ws); \
            } \
        } \
        va_end(list); \
        if (wsallocated != NULL) { \
            free(wsallocated); \
            wsallocated = NULL; \
        } \
        return wr; \
    }

#define DefineRavePrintfA(LEVEL) \
    inline int RavePrintfA ## LEVEL(const log4cxx::LoggerPtr& logger, const log4cxx::spi::LocationInfo& location, const std::string& s) \
    { \
        if (!!logger) { \
            if (s.size() > 0 && s[s.size()-1] == '\n') { \
                std::string s1(s, 0, s.size()-1); \
                OPENRAVE_LOG4CXX ## LEVEL(logger, s1, location); \
            } else { \
                OPENRAVE_LOG4CXX ## LEVEL(logger, s, location); \
            } \
        } else { \
            if (s.size() > 0 && s[s.size()-1] == '\n') { \
                printf("%s", s.c_str()); \
            } else { \
                printf("%s\n", s.c_str()); \
            } \
        } \
        return s.size(); \
    } \
    \
    inline int RavePrintfA ## LEVEL(const log4cxx::LoggerPtr& logger, const log4cxx::spi::LocationInfo& location, const char *fmt, ...) \
    { \
        va_list list; \
        char buf[512]; \
        char* s = &buf[0]; \
        char* sallocated = NULL; \
        int slen = 0; \
        int r = 0; \
        va_start(list,fmt); \
        r = vsnprintf(buf, sizeof(buf)/sizeof(char), fmt, list); \
        if (r >= (int)(sizeof(buf)/sizeof(char))) { \
            slen = r+1; \
            sallocated = (char*)malloc(slen*sizeof(char)); \
            s = sallocated; \
            r = vsnprintf(s, r+1, fmt, list); \
            if (r >= slen) { \
                r = -1; \
            } \
        } \
        if (r >= 0) { \
            /* get rid of the trailing \n if presnet */ \
            if (r > 0 && s[r-1] == '\n') { \
                s[r-1] = '\0'; \
            } \
            if (!!logger) { \
                OPENRAVE_LOG4CXX ## LEVEL(logger, s, location); \
            } else { \
                printf("%s\n", s); \
            } \
        } \
        va_end(list); \
        if (sallocated != NULL) { \
            free(sallocated); \
            sallocated = NULL; \
        } \
        return r; \
    } \

DefineRavePrintfW(_INFOLEVEL)
DefineRavePrintfA(_INFOLEVEL)

inline int RavePrintfA(const std::string& s, uint32_t level)
{
    if( (RaveGetDebugLevel()&Level_OutputMask)>=level ) {
        const log4cxx::LoggerPtr& logger = RaveGetLogger();
        if (!!logger) {
            log4cxx::LevelPtr levelptr = log4cxx::Level::getInfo();
            switch(level&Level_OutputMask) {
            case Level_Fatal: levelptr = log4cxx::Level::getFatal(); break;
            case Level_Error: levelptr = log4cxx::Level::getError(); break;
            case Level_Warn: levelptr = log4cxx::Level::getWarn(); break;
            case Level_Info: levelptr = log4cxx::Level::getInfo(); break;
            case Level_Debug: levelptr = log4cxx::Level::getDebug(); break;
            case Level_Verbose: levelptr = RaveGetVerboseLogLevel(); break;
            }
            if (logger->isEnabledFor(levelptr)) {
                if (s.size() > 0 && s[s.size()-1] == '\n') {
                    std::string s1(s, 0, s.size()-1);
                    logger->forcedLog(levelptr, s1);
                } else {
                    logger->forcedLog(levelptr, s);
                }
            }
        } else {
            if (s.size() > 0 && s[s.size()-1] == '\n') {
                printf("%s", s.c_str());
            } else {
                printf("%s\n", s.c_str());
            }
        }
        return s.size();
    }
    return 0;
}

#define RAVELOG_LOGGER_LEVELW(logger, LEVEL, level, ...) do { if (int(OpenRAVE::RaveGetDebugLevel()&OpenRAVE::Level_OutputMask)>=int(level)) { OpenRAVE::RavePrintfW ## LEVEL(logger, LOG4CXX_LOCATION, __VA_ARGS__); } } while (0)

#define RAVELOG_LOGGER_LEVELA(logger, LEVEL, level, ...) do { if (int(OpenRAVE::RaveGetDebugLevel()&OpenRAVE::Level_OutputMask)>=int(level)) { OpenRAVE::RavePrintfA ## LEVEL(logger, LOG4CXX_LOCATION, __VA_ARGS__); } } while (0)

#undef RAVELOG_LEVELW
#define RAVELOG_LEVELW(LEVEL, level, ...) RAVELOG_LOGGER_LEVELW(OpenRAVE::RaveGetLogger(), LEVEL, level, __VA_ARGS__)

#undef RAVELOG_LEVELA
#define RAVELOG_LEVELA(LEVEL, level, ...) RAVELOG_LOGGER_LEVELA(OpenRAVE::RaveGetLogger(), LEVEL, level, __VA_ARGS__)

#else

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
    if( (RaveGetDebugLevel()&Level_OutputMask)>=level ) {
        int color = 0;
        switch(level&Level_OutputMask) {
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

// define log4cxx equivalents (eventually OpenRAVE will move to log4cxx logging)
#define RAVELOG_FATALW(...) RAVELOG_LEVELW(_FATALLEVEL,OpenRAVE::Level_Fatal,__VA_ARGS__)
#define RAVELOG_FATALA(...) RAVELOG_LEVELA(_FATALLEVEL,OpenRAVE::Level_Fatal,__VA_ARGS__)
#define RAVELOG_FATAL RAVELOG_FATALA
#define RAVELOG_ERRORW(...) RAVELOG_LEVELW(_ERRORLEVEL,OpenRAVE::Level_Error,__VA_ARGS__)
#define RAVELOG_ERRORA(...) RAVELOG_LEVELA(_ERRORLEVEL,OpenRAVE::Level_Error,__VA_ARGS__)
#define RAVELOG_ERROR RAVELOG_ERRORA
#define RAVELOG_WARNW(...) RAVELOG_LEVELW(_WARNLEVEL,OpenRAVE::Level_Warn,__VA_ARGS__)
#define RAVELOG_WARNA(...) RAVELOG_LEVELA(_WARNLEVEL,OpenRAVE::Level_Warn,__VA_ARGS__)
#define RAVELOG_WARN RAVELOG_WARNA
#define RAVELOG_INFOW(...) RAVELOG_LEVELW(_INFOLEVEL,OpenRAVE::Level_Info,__VA_ARGS__)
#define RAVELOG_INFOA(...) RAVELOG_LEVELA(_INFOLEVEL,OpenRAVE::Level_Info,__VA_ARGS__)
#define RAVELOG_INFO RAVELOG_INFOA
#define RAVELOG_DEBUGW(...) RAVELOG_LEVELW(_DEBUGLEVEL,OpenRAVE::Level_Debug,__VA_ARGS__)
#define RAVELOG_DEBUGA(...) RAVELOG_LEVELA(_DEBUGLEVEL,OpenRAVE::Level_Debug,__VA_ARGS__)
#define RAVELOG_DEBUG RAVELOG_DEBUGA
#define RAVELOG_VERBOSEW(...) RAVELOG_LEVELW(_VERBOSELEVEL,OpenRAVE::Level_Verbose,__VA_ARGS__)
#define RAVELOG_VERBOSEA(...) RAVELOG_LEVELA(_VERBOSELEVEL,OpenRAVE::Level_Verbose,__VA_ARGS__)
#define RAVELOG_VERBOSE RAVELOG_VERBOSEA

#define RAVELOG_FATAL_FORMAT(x, params) RAVELOG_FATAL(boost::str(boost::format(x)%params))
#define RAVELOG_ERROR_FORMAT(x, params) RAVELOG_ERROR(boost::str(boost::format(x)%params))
#define RAVELOG_WARN_FORMAT(x, params) RAVELOG_WARN(boost::str(boost::format(x)%params))
#define RAVELOG_INFO_FORMAT(x, params) RAVELOG_INFO(boost::str(boost::format(x)%params))
#define RAVELOG_DEBUG_FORMAT(x, params) RAVELOG_DEBUG(boost::str(boost::format(x)%params))
#define RAVELOG_VERBOSE_FORMAT(x, params) RAVELOG_VERBOSE(boost::str(boost::format(x)%params))

#define IS_DEBUGLEVEL(level) ((OpenRAVE::RaveGetDebugLevel()&OpenRAVE::Level_OutputMask)>=(level))

}

#endif
