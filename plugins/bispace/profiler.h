#ifndef SMALL_PROFILER_H
#define SMALL_PROFILER_H

#include <assert.h>
#include <stdint.h>

////
// profiling
///

extern int g_bWriteProfile; // global variable to enable/disable profiling (if DVPROFILE is defined)

// IMPORTANT: For every Reigster there must be an End
void DVProfRegister(const char* pname);			// first checks if this profiler exists in g_listProfilers
void DVProfEnd(uint32_t dwUserData);
void DVProfWrite(const char* pfilename, uint32_t frames = 1);
void DVProfClear();						// clears all the profilers

#if defined(DVPROFILE)

#ifdef _MSC_VER

#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __FUNCTION__
#endif

#endif

#define DVSTARTPROFILE() DVProfileFunc _pf(__FUNCTION__);

class DVProfileFunc
{
public:
	uint32_t dwUserData;
	DVProfileFunc(const char* pname) { DVProfRegister(pname); dwUserData = 0; }
	DVProfileFunc(const char* pname, uint32_t dwUserData) : dwUserData(dwUserData) { DVProfRegister(pname); }
	~DVProfileFunc() { DVProfEnd(dwUserData); }
};

#else

#define DVSTARTPROFILE()

class DVProfileFunc
{
public:
	uint32_t dwUserData;
    inline DVProfileFunc(const char* pname) {}
    inline DVProfileFunc(const char* pname, uint32_t dwUserData) { }
	~DVProfileFunc() {}
};

#endif

#endif
