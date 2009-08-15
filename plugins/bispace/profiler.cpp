#include "plugindefs.h"

////////////////////
// Small profiler //
////////////////////
#include <list>
#include <string>
#include <map>

using namespace std;

int g_bWriteProfile=1;

#ifdef _WIN32

//#if defined (_MSC_VER) && _MSC_VER >= 1400
//#if _MSC_VER < 1500
//#include <intrin.h>
//#endif
//#pragma intrinsic(__rdtsc)
//#endif

static uint64_t luPerfFreq=0;
inline uint64_t GET_PROFILE_TIME()
{
//#if defined (_MSC_VER) && _MSC_VER >= 1400
//    return __rdtsc();
//#else
    LARGE_INTEGER lu;
    QueryPerformanceCounter(&lu);
    return lu.QuadPart;
//#endif
}
#else

#include <sys/timeb.h>    // ftime(), struct timeb
#include <sys/time.h>

static uint64_t luPerfFreq = 1000000;
inline uint64_t GET_PROFILE_TIME()
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return (uint64_t)t.tv_sec*1000000+t.tv_usec;
}
#endif

struct DVPROFSTRUCT
{
    struct DATA
    {
        DATA(uint64_t time, uint32_t user = 0) : dwTime(time), dwUserData(user) {}
        DATA() : dwTime(0), dwUserData(0) {}
        
        uint64_t dwTime;
        uint32_t dwUserData;
    };

    ~DVPROFSTRUCT() {
        list<DVPROFSTRUCT*>::iterator it = listpChild.begin();
        while(it != listpChild.end() ) {
            delete *it; *it = NULL;
            ++it;
        }
    }

    list<DATA> listTimes;       // before DVProfEnd is called, contains the global time it started
                                // after DVProfEnd is called, contains the time it lasted
                                // the list contains all the tracked times 
    char pname[256];

    list<DVPROFSTRUCT*> listpChild;     // other profilers called during this profiler period
};

struct DVPROFTRACK
{
    uint32_t dwUserData;
    DVPROFSTRUCT::DATA* pdwTime;
    DVPROFSTRUCT* pprof;
};

list<DVPROFTRACK> g_listCurTracking;    // the current profiling functions, the back element is the
                                        // one that will first get popped off the list when DVProfEnd is called
                                        // the pointer is an element in DVPROFSTRUCT::listTimes
list<DVPROFSTRUCT> g_listProfilers;         // the current profilers, note that these are the parents
                                            // any profiler started during the time of another is held in
                                            // DVPROFSTRUCT::listpChild
list<DVPROFSTRUCT*> g_listAllProfilers;     // ignores the hierarchy, pointer to elements in g_listProfilers

void DVProfRegister(const char* pname)
{
    if( !g_bWriteProfile )
        return;
    list<DVPROFSTRUCT*>::iterator it = g_listAllProfilers.begin();
    
//  while(it != g_listAllProfilers.end() ) {
//
//      if( _tcscmp(pname, (*it)->pname) == 0 ) {
//          (*it)->listTimes.push_back(timeGetTime());
//          DVPROFTRACK dvtrack;
//          dvtrack.pdwTime = &(*it)->listTimes.back();
//          dvtrack.pprof = *it;
//          g_listCurTracking.push_back(dvtrack);
//          return;
//      }
//
//      ++it;
//  }

    // else add in a new profiler to the appropriate parent profiler
    DVPROFSTRUCT* pprof = NULL;
    
    if( g_listCurTracking.size() > 0 ) {
        assert( g_listCurTracking.back().pprof != NULL );
        g_listCurTracking.back().pprof->listpChild.push_back(new DVPROFSTRUCT());
        pprof = g_listCurTracking.back().pprof->listpChild.back();
    }
    else {
        g_listProfilers.push_back(DVPROFSTRUCT());
        pprof = &g_listProfilers.back();
    }

    strncpy(pprof->pname, pname, 256);

    // setup the profiler for tracking
    pprof->listTimes.push_back(DVPROFSTRUCT::DATA(GET_PROFILE_TIME()));

    DVPROFTRACK dvtrack;
    dvtrack.pdwTime = &pprof->listTimes.back();
    dvtrack.pprof = pprof;
    dvtrack.dwUserData = 0;

    g_listCurTracking.push_back(dvtrack);

    // add to all profiler list
    g_listAllProfilers.push_back(pprof);
}

void DVProfEnd(uint32_t dwUserData)
{
    if( !g_bWriteProfile )
        return;
    if( g_listCurTracking.size() == 0 )
        return;

    DVPROFTRACK dvtrack = g_listCurTracking.back();

    assert( dvtrack.pdwTime != NULL && dvtrack.pprof != NULL );

    dvtrack.pdwTime->dwTime = GET_PROFILE_TIME()- dvtrack.pdwTime->dwTime;
    dvtrack.pdwTime->dwUserData= dwUserData;

    g_listCurTracking.pop_back();
}

struct DVTIMEINFO
{
    DVTIMEINFO() : uInclusive(0), uExclusive(0) {}
    uint64_t uInclusive, uExclusive;
};

map<string, DVTIMEINFO> mapAggregateTimes;

uint64_t DVProfWriteStruct(FILE* f, DVPROFSTRUCT* p, int ident)
{
    fprintf(f, "%*s%s - ", ident, "", p->pname);

    list<DVPROFSTRUCT::DATA>::iterator ittime = p->listTimes.begin();

    uint64_t utime = 0;

    while(ittime != p->listTimes.end() ) {
//        if( ittime->dwTime > luPerfFreq*10 ) {
//            ++ittime;
//            continue;
//        }
        
        utime += ittime->dwTime;
        
        if( ittime->dwUserData ) 
            fprintf(f, "time: %d, user: 0x%8.8x", (uint32_t)ittime->dwTime, ittime->dwUserData);
        else
            fprintf(f, "time: %d", (uint32_t)ittime->dwTime);
        ++ittime;
    }

    // yes this is necessary, maps have problems with constructors on their type
    map<string, DVTIMEINFO>::iterator ittimes = mapAggregateTimes.find(p->pname);
    if( ittimes == mapAggregateTimes.end() ) {
        ittimes = mapAggregateTimes.insert(map<string, DVTIMEINFO>::value_type(p->pname, DVTIMEINFO())).first;
        ittimes->second.uExclusive = 0;
        ittimes->second.uInclusive = 0;
    }

    ittimes->second.uInclusive += utime;

    fprintf(f, "\n");

    list<DVPROFSTRUCT*>::iterator itprof = p->listpChild.begin();

    uint64_t uex = utime;
    while(itprof != p->listpChild.end() ) {
        uex -= DVProfWriteStruct(f, *itprof, ident+4);
        ++itprof;
    }

    if( uex > utime ) {
        uex = 0;
        //RAVEPRINT(L"profiling precision error!\n");
    }

    ittimes->second.uExclusive += uex;
    return utime;
}

void DVProfWrite(const char* pfilename, uint32_t frames)
{
    assert( pfilename != NULL );
    FILE* f = fopen(pfilename, "w");

    // pop back any unused
    mapAggregateTimes.clear();
    list<DVPROFSTRUCT>::iterator it = g_listProfilers.begin();

    while(it != g_listProfilers.end() ) {
        DVProfWriteStruct(f, &(*it), 0);
        ++it;
    }

    {
        map<string, DVTIMEINFO>::iterator it;
        fprintf(f, "\n\n--------------------------------------------------------------\n\n");

        uint64_t uTotal[2]; uTotal[0] = uTotal[1] = 0;
        double fiTotalTime[2]; fiTotalTime[0] = fiTotalTime[1] = 0;

        for(it = mapAggregateTimes.begin(); it != mapAggregateTimes.end(); ++it) {
            uTotal[0] += it->second.uExclusive;
            uTotal[1] += it->second.uInclusive;
        }

        fprintf(f, "total times (%d): ex: %llu ", frames, 1000000*uTotal[0]/(luPerfFreq*(uint64_t)frames));
        fprintf(f, "inc: %llu\n", 1000000 * uTotal[1]/(luPerfFreq*(uint64_t)frames));

        if( uTotal[0] > 0 )
            fiTotalTime[0] = 1.0 / (double)uTotal[0];
        if( uTotal[1] > 0 )
            fiTotalTime[1] = 1.0 / (double)uTotal[1];

        // output the combined times
        for(it = mapAggregateTimes.begin(); it != mapAggregateTimes.end(); ++it) {
            fprintf(f, "%s - ex: %f%% (%fs) inc: %f%%\n", it->first.c_str(),
                    100.0f*(float)((double)it->second.uExclusive * fiTotalTime[0]),
                    (double)it->second.uExclusive/(double)luPerfFreq,
                    100.0f*(float)((double)it->second.uInclusive * fiTotalTime[1]));
        }
    }


    fclose(f);
}

void DVProfClear()
{
    g_listCurTracking.clear();
    g_listProfilers.clear();
    g_listAllProfilers.clear();
}
