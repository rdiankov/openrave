#include <rave/rave.h>

#ifndef _WIN32
#define strnicmp strncasecmp
#define stricmp strcasecmp
#endif

#if defined(__APPLE_CC__)
inline int wcsicmp(const wchar_t* s1, const wchar_t* s2)
{
  char str1[128], str2[128];
  sprintf(str1, "%S", s1);
  sprintf(str2, "%S", s2);
  return stricmp(str1, str2);
}
#elif !defined(_WIN32)
#define wcsnicmp wcsncasecmp
#define wcsicmp wcscasecmp
#endif

using namespace std;
using namespace OpenRAVE;

class MyProblemInstance : public CmdProblemInstance
{
public:
    MyProblemInstance(EnvironmentBase* penv) : CmdProblemInstance(penv)
    {
        RegisterCommand("numbodies",(CommandFn)&MyProblemInstance::NumBodies, "returns bodies");
        RegisterCommand("load",(CommandFn)&MyProblemInstance::Load, "loads a given file");
    }

    void Destroy() {
        RAVELOG_INFOA("problem unloaded from environment\n");
    }

    int main(const char* cmd)
    {
        RAVELOG_INFOA("problem initialized cmd; %s\n", cmd);
        return 0;
    }

    bool NumBodies(ostream& sout, istream& sinput)
    {
        sout << GetEnv()->GetBodies().size();
        return true;
    }

    bool Load(ostream& sout, istream& sinput)
    {
        string filename;
        sinput >> filename;
        bool bSuccess = GetEnv()->Load(filename.c_str()); // load the file
        sout << bSuccess; // publish the results
        return true;
    }
};

#ifdef _MSC_VER
#define PROT_STDCALL(name, paramlist) __stdcall name paramlist
#define DECL_STDCALL(name, paramlist) __stdcall name paramlist
#else
#ifdef __x86_64__
#define DECL_STDCALL(name, paramlist) name paramlist
#else
#define DECL_STDCALL(name, paramlist) __attribute__((stdcall)) name paramlist
#endif
#endif // _MSC_VER

// for some reason windows complains when the prototypes are different
extern "C" InterfaceBase* DECL_STDCALL(ORCreate, (PluginType type, wchar_t* name, EnvironmentBase* penv))
{
    if( name == NULL ) return NULL;
    
    switch(type) {
        case PT_ProblemInstance:
            if( wcsicmp(name, L"MyProblem") == 0 )
                return new MyProblemInstance(penv);
            break;
        default:
            break;
    }

    return NULL;
}

extern "C" bool DECL_STDCALL(GetPluginAttributes, (PLUGININFO* pinfo, int size))
{
    if( pinfo == NULL ) return false;
    if( size != sizeof(PLUGININFO) ) {
        printf("bad plugin info sizes %d != %d\n", size, sizeof(PLUGININFO));
        return false;
    }

    // fill pinfo
    pinfo->problems.push_back(L"MyProblem");
    return true;
}

extern "C"void DECL_STDCALL(DestroyPlugin, ())
{
    RAVELOG_INFOA("destroying plugincpp");
}
