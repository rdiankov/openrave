#ifndef OPENRAVE_IKFAST_PROBLEM
#define OPENRAVE_IKFAST_PROBLEM

#include "plugindefs.h"
#include <boost/algorithm/string.hpp>

#ifdef Boost_IOSTREAMS_FOUND
#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/stream.hpp>
#endif

#include <errno.h>
#include <stdio.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <dlfcn.h>
#include <sys/types.h>
#include <dirent.h>
#endif

#define DECLFNPTR(name, paramlist) (*name) paramlist
#define IK2PI  6.28318530717959
#define IKPI  3.14159265358979
#define IKPI_2  1.57079632679490

class IKSolutionFloat
{
 public:
    typedef float IKReal;
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
    typedef double IKReal;
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
    typedef int DECLFNPTR(getIKTypeFn, ());
    static int getDefaultIKType() { return IkParameterization::Type_Transform6D; }

    class IKLibrary
    {
    public:
        IKLibrary() : plib(NULL) {}
        ~IKLibrary() {
            if( plib != NULL ) {
                SysCloseLibrary(plib);
            }
        }

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
            getIKType = (getIKTypeFn)SysLoadSym(plib, "getIKType");
            if( getIKType == NULL ) {
                RAVELOG_WARNA("failed to find getIKType in %s, setting to 6D transform\n", _libraryname.c_str());
                getIKType = getDefaultIKType;
                //return false;
            }
            ikfn = SysLoadSym(plib, "ik");
            if( ikfn == NULL ) {
                RAVELOG_WARNA("failed to find ik in %s\n", _libraryname.c_str());
                return false;
            }
            fkfn = SysLoadSym(plib, "fk");

            vfree.resize(getNumFreeParameters());
            for(size_t i = 0; i < vfree.size(); ++i)
                vfree[i] = getFreeParameters()[i];
            return true;
        }

        IkSolverBasePtr CreateSolver(EnvironmentBasePtr penv, dReal ffreedelta)
        {
            if( getIKRealSize() == 4 )
                return IkSolverBasePtr(new IkFastSolver<float,IKSolutionFloat >((IkFastSolver<float,IKSolutionFloat >::IkFn)ikfn,vfree,ffreedelta,getNumJoints(),(IkParameterization::Type)getIKType(),penv));
            else if( getIKRealSize() == 8 )
                return IkSolverBasePtr(new IkFastSolver<double,IKSolutionDouble >((IkFastSolver<double,IKSolutionDouble >::IkFn)ikfn,vfree,ffreedelta,getNumJoints(),(IkParameterization::Type)getIKType(),penv));
            throw openrave_exception("bad real size");
        }

        const string& GetIKName() { return _ikname; }
        const string& GetLibraryName() { return _libraryname; }
        int GetIKType() { return getIKType(); }

        getNumFreeParametersFn getNumFreeParameters;
        getFreeParametersFn getFreeParameters;
        getNumJointsFn getNumJoints;
        getIKRealSizeFn getIKRealSize;
        getIKTypeFn getIKType;
        void* ikfn, *fkfn;

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
        string _ikname, _libraryname;
        vector<int> vfree;
    };

    inline boost::shared_ptr<IKFastProblem> shared_problem() { return boost::static_pointer_cast<IKFastProblem>(shared_from_this()); }
    inline boost::shared_ptr<IKFastProblem const> shared_problem_const() const { return boost::static_pointer_cast<IKFastProblem const>(shared_from_this()); }

public:
    IKFastProblem(EnvironmentBasePtr penv) : ProblemInstance(penv)
    {
        RegisterCommand("AddIkLibrary",boost::bind(&IKFastProblem::AddIkLibrary,this,_1,_2),
                        "Dynamically adds an ik solver to openrave by loading a shared object (based on ikfast code generation).\n"
                        "Usage:\n    AddIkLibrary iksolvername iklibrarypath\n"
                        "return the type of inverse kinematics solver (IkParamterization::Type)");
#ifdef Boost_IOSTREAMS_FOUND
        RegisterCommand("LoadIKFastSolver",boost::bind(&IKFastProblem::LoadIKFastSolver,this,_1,_2),
                        "Dynamically calls the inversekinematics.py script to generate an ik solver for a robot, or to load an existing one\n"
                        "Usage:\n    LoadIKFastSolver robotname iktype_id [free increment]\n"
                        "return nothing, but does call the SetIKSolver for the robot");
#endif
        RegisterCommand("PerfTiming",boost::bind(&IKFastProblem::PerfTiming,this,_1,_2),
                        "Times the ik call of a given library.\n"
                        "Usage:\n    PerfTiming [options] iklibrarypath\n"
                        "return the set of time measurements made");
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
        if( sinput.eof() ) {
            return false;
        }
        string ikname, libraryname;
        sinput >> ikname;
        std::transform(ikname.begin(), ikname.end(), ikname.begin(), ::tolower);
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
        sout << lib->GetIKType();
        _vlibraries.push_back(lib);
        return true;
    }

#ifdef Boost_IOSTREAMS_FOUND
    bool LoadIKFastSolver(ostream& sout, istream& sinput)
    {
        EnvironmentMutex::scoped_lock envlock(GetEnv()->GetMutex());
        string robotname, striktype;
        int niktype;
        sinput >> robotname;
        sinput >> niktype;
        if( !sinput ) {
            return false;
        }
        RobotBasePtr probot = GetEnv()->GetRobot(robotname);
        if( !probot || !probot->GetActiveManipulator() ) {
            return false;
        }
        switch (static_cast<IkParameterization::Type>(niktype)) {
            case IkParameterization::Type_Transform6D: striktype="Transform6D"; break;
            case IkParameterization::Type_Rotation3D: striktype="Rotation3D"; break;
            case IkParameterization::Type_Translation3D: striktype="Translation3D"; break;
            case IkParameterization::Type_Direction3D: striktype="Direction3D"; break;
            case IkParameterization::Type_Ray4D: striktype="Ray4D"; break;
            default:
                return false;
        }

        {
            string cmdhas = str(boost::format("openrave.py --database=\"inversekinematics --gethas --robot=%s --manipname=%s --iktype=%s\"")%probot->GetXMLFilename()%probot->GetActiveManipulator()->GetName()%striktype);
            FILE* pipe = popen(cmdhas.c_str(), "r");
            int has = pclose(pipe);
            if( has ) {
                string cmdgen = str(boost::format("openrave.py --database=\"inversekinematics --robot=%s --manipname=%s --iktype=%s\"")%probot->GetXMLFilename()%probot->GetActiveManipulator()->GetName()%striktype);
                FILE* pipe = popen(cmdgen.c_str(), "r");
                int generateexit = pclose(pipe);
                RAVELOG_INFO("generate exit: %d\n",generateexit);
            }
        }
        
        string cmdfilename = str(boost::format("openrave.py --database=\"inversekinematics --getfilename --robot=%s --manipname=%s --iktype=%s\"")%probot->GetXMLFilename()%probot->GetActiveManipulator()->GetName()%striktype);
        RAVELOG_INFO("executing shell command:\n%s\n",cmdfilename.c_str());
        string ikfilename;
        FILE* pipe = popen(cmdfilename.c_str(), "r");
        {
            boost::iostreams::stream_buffer<boost::iostreams::file_descriptor_source> fpstream(fileno(pipe));
            std::istream in(&fpstream);
            std::getline(in, ikfilename);
            if( !in ) {
                pclose(pipe);
                return false;
            }
        }
        pclose(pipe);

        string ikfastname = str(boost::format("ikfast.%s.%s")%probot->GetRobotStructureHash()%probot->GetActiveManipulator()->GetName());
        boost::shared_ptr<IKLibrary> lib(new IKLibrary());
        if( !lib->Init(ikfastname, ikfilename) ) {
            return false;
        }
        if( lib->GetIKType() != niktype ) {
            return false;
        }
        IkSolverBasePtr iksolver = lib->CreateSolver(GetEnv(), 0.1);
        if( !iksolver ) {
            return false;
        }
        _vlibraries.push_back(lib);
        probot->GetActiveManipulator()->SetIKSolver(iksolver);
        return probot->GetActiveManipulator()->InitIKSolver();
    }
#endif
    
    bool PerfTiming(ostream& sout, istream& sinput)
    {
        string cmd, libraryname;
        int num=100;
        while(!sinput.eof()) {
            istream::streampos pos = sinput.tellg();
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "num" ) {
                sinput >> num;
            }
            else {
                sinput.clear(); // have to clear eof bit
                sinput.seekg(pos);
                getline(sinput, libraryname);
                break;
            }
        }

        boost::trim(libraryname);
        if( libraryname.size() == 0 ) {
            return false;
        }
        boost::shared_ptr<IKLibrary> lib(new IKLibrary());
        if( !lib->Init("", libraryname) ) {
            RAVELOG_WARN(str(boost::format("failed to init library %s\n")%libraryname));
            return false;
        }

        if( lib->getIKRealSize() == 4 ) {
            return _PerfTiming<IKSolutionFloat>(sout,lib,num);
        }
        else if( lib->getIKRealSize() == 8 ) {
            return _PerfTiming<IKSolutionDouble>(sout,lib,num);
        }
        else {
            throw openrave_exception("bad real size");
        }
        return true;
    }

    template<typename T> bool _PerfTiming(ostream& sout, boost::shared_ptr<IKLibrary> lib, int num)
    {
        BOOST_ASSERT(lib->getIKRealSize()==sizeof(typename T::IKReal));
        typename IkFastSolver<typename T::IKReal,T>::IkFn ikfn = (typename IkFastSolver<typename T::IKReal,T>::IkFn)lib->ikfn;
        typename IkFastSolver<typename T::IKReal,T>::FkFn fkfn = (typename IkFastSolver<typename T::IKReal,T>::FkFn)lib->fkfn;
        if( !ikfn || !fkfn )
            return false;
        vector<uint64_t> vtimes(num);
        vector<T> vsolutions; vsolutions.reserve(32);
        vector<typename T::IKReal> vjoints(lib->getNumJoints()), vfree(lib->getNumFreeParameters());
        typename T::IKReal eerot[9],eetrans[3];
        for(size_t i = 0; i < vtimes.size(); ++i) {
            for(size_t j = 0; j < vjoints.size(); ++j) {
                vjoints[j] = RaveRandomDouble()*2*PI;
            }
            for(size_t j = 0; j < vfree.size(); ++j) {
                vfree[j] = vjoints[lib->getFreeParameters()[j]];
            }
            fkfn(&vjoints[0],eetrans,eerot);
            vsolutions.resize(0);

            uint64_t numtoaverage=10;
            uint64_t starttime = GetNanoTime();
            for(uint64_t j = 0; j < numtoaverage; ++j) {
                ikfn(eetrans,eerot,vfree.size() > 0 ? &vfree[0] : NULL,vsolutions);
            }
            vtimes[i] = (GetNanoTime()-starttime)/numtoaverage;
        }
        FOREACH(it,vtimes) {
            sout << ((*it)/1000) << " ";
        }
        return true;
    }
    
    static vector< boost::shared_ptr<IKFastProblem> >& GetProblems()
    {
        static vector< boost::shared_ptr<IKFastProblem> > s_vStaticProblems;
        return s_vStaticProblems;
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
