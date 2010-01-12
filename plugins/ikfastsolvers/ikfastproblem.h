#ifndef OPENRAVE_IKFAST_PROBLEM
#define OPENRAVE_IKFAST_PROBLEM

#include "plugindefs.h"
#include <boost/algorithm/string.hpp>

#include <errno.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define PLUGIN_EXT ".dll"
#else
#include <dlfcn.h>
#include <sys/types.h>
#include <dirent.h>

#ifdef __APPLE_CC__
#define PLUGIN_EXT ".dylib"
#else
#define PLUGIN_EXT ".so"
#endif

#endif

#define DECLFNPTR(name, paramlist) (*name) paramlist

#define IK2PI  6.28318530717959
#define IKPI  3.14159265358979
#define IKPI_2  1.57079632679490

class IKSolutionFloat
{
 public:
    /// Gets a solution given its free parameters
    /// \param pfree The free parameters required, range is in [-pi,pi]
    void GetSolution(float* psolution, const float* pfree) const {
        for(size_t i = 0; i < basesol.size(); ++i) {
            if( basesol[i].freeind < 0 )
                psolution[i] = basesol[i].foffset;
            else {
                BOOST_ASSERT(pfree != NULL);
                psolution[i] = pfree[basesol[i].freeind]*basesol[i].fmul + basesol[i].foffset;
                if( psolution[i] > IKPI )
                    psolution[i] -= IK2PI;
                else if( psolution[i] < -IKPI )
                    psolution[i] += IK2PI;
            }
        }
    }

    /// Gets the free parameters the solution requires to be set before a full solution can be returned
    /// \return vector of indices indicating the free parameters
    const std::vector<int>& GetFree() const { return vfree; }

    struct VARIABLE
    {
    VARIABLE() : freeind(-1), fmul(0), foffset(0) {}
    VARIABLE(int freeind, float fmul, float foffset) : freeind(freeind), fmul(fmul), foffset(foffset) {}
        int freeind;
        float fmul, foffset; ///< joint value is fmul*sol[freeind]+foffset
    };

    std::vector<VARIABLE> basesol;       ///< solution and their offsets if joints are mimiced
    std::vector<int> vfree;
};
class IKSolutionDouble
{
 public:
    /// Gets a solution given its free parameters
    /// \param pfree The free parameters required, range is in [-pi,pi]
    void GetSolution(double* psolution, const double* pfree) const {
        for(size_t i = 0; i < basesol.size(); ++i) {
            if( basesol[i].freeind < 0 )
                psolution[i] = basesol[i].foffset;
            else {
                BOOST_ASSERT(pfree != NULL);
                psolution[i] = pfree[basesol[i].freeind]*basesol[i].fmul + basesol[i].foffset;
                if( psolution[i] > IKPI )
                    psolution[i] -= IK2PI;
                else if( psolution[i] < -IKPI )
                    psolution[i] += IK2PI;
            }
        }
    }

    /// Gets the free parameters the solution requires to be set before a full solution can be returned
    /// \return vector of indices indicating the free parameters
    const std::vector<int>& GetFree() const { return vfree; }

    struct VARIABLE
    {
    VARIABLE() : freeind(-1), fmul(0), foffset(0) {}
    VARIABLE(int freeind, double fmul, double foffset) : freeind(freeind), fmul(fmul), foffset(foffset) {}
        int freeind;
        double fmul, foffset; ///< joint value is fmul*sol[freeind]+foffset
    };

    std::vector<VARIABLE> basesol;       ///< solution and their offsets if joints are mimiced
    std::vector<int> vfree;
};

class IKFastProblem : public ProblemInstance
{
    typedef int DECLFNPTR(getNumFreeParametersFn, ());
    typedef int* DECLFNPTR(getFreeParametersFn, ());
    typedef int DECLFNPTR(getNumJointsFn, ());
    typedef int DECLFNPTR(getIKRealSizeFn, ());

    class IKLibrary
    {
    public:
        IKLibrary() : plib(NULL) {}
        ~IKLibrary() { if( plib != NULL ) SysCloseLibrary(plib); }

        bool Init(const string& ikname, const string& libraryname)
        {
            _ikname = ikname;
            _libraryname = libraryname;
            plib = SysLoadLibrary(_libraryname.c_str());
            if( plib == NULL ) {
                RAVELOG_WARNA("failed to load library %s\n", _libraryname.c_str());
                return false;
            }

            getNumFreeParameters = (getNumFreeParametersFn)SysLoadSym(plib, "getNumFreeParameters");
            if( getNumFreeParameters == NULL ) {
                RAVELOG_WARNA("failed to find getNumFreeParameters in %s\n", _libraryname.c_str());
                return false;
            }
            getFreeParameters = (getFreeParametersFn)SysLoadSym(plib, "getFreeParameters");
            if( getFreeParameters == NULL ) {
                RAVELOG_WARNA("failed to find getFreeParameters in %s\n", _libraryname.c_str());
                return false;
            }
            getNumJoints = (getNumJointsFn)SysLoadSym(plib, "getNumJoints");
            if( getNumJoints == NULL ) {
                RAVELOG_WARNA("failed to find getNumJoints in %s\n", _libraryname.c_str());
                return false;
            }
            getIKRealSize = (getIKRealSizeFn)SysLoadSym(plib, "getIKRealSize");
            if( getIKRealSize == NULL ) {
                RAVELOG_WARNA("failed to find getIKRealSize in %s\n", _libraryname.c_str());
                return false;
            }
            ikfn = SysLoadSym(plib, "ik");
            if( ikfn == NULL ) {
                RAVELOG_WARNA("failed to find ik in %s\n", _libraryname.c_str());
                return false;
            }

            vfree.resize(getNumFreeParameters());
            for(size_t i = 0; i < vfree.size(); ++i)
                vfree[i] = getFreeParameters()[i];
            return true;
        }

        IkSolverBasePtr CreateSolver(EnvironmentBasePtr penv, dReal ffreedelta=0.04f)
        {
            if( getIKRealSize() == 4 )
                return IkSolverBasePtr(new IkFastSolver<float,IkSolverBase::Parameterization::Type_Transform6D,IKSolutionFloat >((IkFastSolver<float,IkSolverBase::Parameterization::Type_Transform6D,IKSolutionFloat >::IkFn)ikfn,vfree,ffreedelta,getNumJoints(),penv));
            else if( getIKRealSize() == 8 )
                return IkSolverBasePtr(new IkFastSolver<double,IkSolverBase::Parameterization::Type_Transform6D,IKSolutionDouble >((IkFastSolver<double,IkSolverBase::Parameterization::Type_Transform6D,IKSolutionDouble >::IkFn)ikfn,vfree,ffreedelta,getNumJoints(),penv));
            throw openrave_exception("bad real size");
        }

        const string& GetIKName() { return _ikname; }
        const string& GetLibraryName() { return _libraryname; }

    private:
        void* SysLoadLibrary(const char* lib)
        {
#ifdef _WIN32
            void* plib = LoadLibraryA(lib);
            if( plib == NULL ) {
                RAVELOG_WARNA("Failed to load %s\n", lib);
            }
#else
            void* plib = dlopen(lib, RTLD_NOW);
            if( plib == NULL ) {
                RAVELOG_WARNA("%s\n", dlerror());
            }
#endif
            return plib;
        }

        void* SysLoadSym(void* lib, const char* sym)
        {
#ifdef _WIN32
            return GetProcAddress((HINSTANCE)lib, sym);
#else
            return dlsym(lib, sym);
#endif
        }

        void SysCloseLibrary(void* lib)
        {
#ifdef _WIN32
            FreeLibrary((HINSTANCE)lib);
#else
            dlclose(lib);
#endif
        }
    
        void* plib;
        getNumFreeParametersFn getNumFreeParameters;
        getFreeParametersFn getFreeParameters;
        getNumJointsFn getNumJoints;
        getIKRealSizeFn getIKRealSize;
        void* ikfn;
        string _ikname, _libraryname;
        vector<int> vfree;
    };

    inline boost::shared_ptr<IKFastProblem> shared_problem() { return boost::static_pointer_cast<IKFastProblem>(shared_from_this()); }
    inline boost::shared_ptr<IKFastProblem const> shared_problem_const() const { return boost::static_pointer_cast<IKFastProblem const>(shared_from_this()); }

public:
    IKFastProblem(EnvironmentBasePtr penv) : ProblemInstance(penv)
    {
        RegisterCommand("AddIkLibrary",boost::bind(&IKFastProblem::AddIkLibrary,this,_1,_2),
                        "Dynamically adds an ik solver to openrave (based on ikfast code generation).\nUsage:\n    AddIkLibrary iksolvername iklibrarypath");
    }

    virtual ~IKFastProblem() {}

    int main(const string& cmd)
    {
        GetProblems().push_back(shared_problem());

        return 0;
    }

    virtual void Destroy()
    {
        GetProblems().erase(find(GetProblems().begin(),GetProblems().end(),shared_problem()));   
    }

    bool AddIkLibrary(ostream& sout, istream& sinput)
    {
        if( sinput.eof() )
            return false;
        string ikname, libraryname;
        sinput >> ikname;
        if (!getline(sinput, libraryname) )
            return false;
        boost::trim(libraryname);
        if( !sinput || libraryname.size() == 0 || ikname.size() == 0 ) {
            RAVELOG_DEBUGA("bad input\n");
            return false;
        }
        boost::shared_ptr<IKLibrary> lib(new IKLibrary());
        if( !lib->Init(ikname, libraryname) )
            return false;

        _vlibraries.push_back(lib);
        return true;
    }

    IkSolverBasePtr CreateIkSolver(const string& name, dReal freeinc, EnvironmentBasePtr penv)
    {
        /// start from the newer libraries
        for(vector< boost::shared_ptr<IKLibrary> >::reverse_iterator itlib = _vlibraries.rbegin(); itlib != _vlibraries.rend(); ++itlib) {
            if( name == (*itlib)->GetIKName() )
                return (*itlib)->CreateSolver(penv,freeinc);
        }
        return IkSolverBasePtr();
    }
    
    static vector< boost::shared_ptr<IKFastProblem> >& GetProblems()
    {
        static vector< boost::shared_ptr<IKFastProblem> > s_vStaticProblems;
        return s_vStaticProblems;
    }

private:
    vector< boost::shared_ptr<IKLibrary> > _vlibraries;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(IKSolutionFloat)
BOOST_TYPEOF_REGISTER_TYPE(IKSolutionDouble)
BOOST_TYPEOF_REGISTER_TYPE(IKFastProblem)
#endif

#endif
