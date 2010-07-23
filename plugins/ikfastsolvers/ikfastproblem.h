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

#ifdef HAVE_BOOST_FILESYSTEM
#include <boost/filesystem/operations.hpp>
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

    class IKLibrary : public boost::enable_shared_from_this<IKLibrary>
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
            _viknames.resize(1); _viknames[0] = ikname;
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

        const vector<string>& GetIKNames() const { return _viknames; }
        void AddIKName(const string& ikname) { _viknames.push_back(ikname); }
        const string& GetLibraryName() const { return _libraryname; }
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
        string _libraryname;
        vector<string> _viknames;
        vector<int> vfree;
    };

    inline boost::shared_ptr<IKFastProblem> shared_problem() { return boost::static_pointer_cast<IKFastProblem>(shared_from_this()); }
    inline boost::shared_ptr<IKFastProblem const> shared_problem_const() const { return boost::static_pointer_cast<IKFastProblem const>(shared_from_this()); }

public:
    IKFastProblem(EnvironmentBasePtr penv) : ProblemInstance(penv)
    {
        __description = ":Interface Author: Rosen Diankov\nAllows dynamic loading and registering of ikfast shared objects to openrave plugins.\nAlso contains several test routines for inverse kinematics.";
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
        RegisterCommand("IKTest",boost::bind(&IKFastProblem::IKtest,this,_1,_2),
                        "Tests for an IK solution if active manipulation has an IK solver attached");
        RegisterCommand("DebugIK",boost::bind(&IKFastProblem::DebugIK,this,_1,_2),
                        "Function used for debugging and testing an IK solver");

    }

    virtual ~IKFastProblem() {}

    int main(const string& cmd)
    {
        return 0;
    }

    virtual void Destroy()
    {
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

        boost::shared_ptr<IKLibrary> lib = _AddIkLibrary(ikname,libraryname);
        if( !lib ) {
            return false;
        }
        sout << lib->GetIKType();
        return true;
    }

    boost::shared_ptr<IKLibrary> _AddIkLibrary(const string& ikname, const string& _libraryname)
    {
#ifdef HAVE_BOOST_FILESYSTEM
        string libraryname = boost::filesystem::system_complete(boost::filesystem::path(_libraryname, boost::filesystem::native)).string();
#else
        string libraryname=_libraryname;
#endif

        // before adding a new library, check for existing
        boost::mutex::scoped_lock lock(GetLibraryMutex());
        boost::shared_ptr<IKLibrary> lib;
        FOREACH(it, GetLibraries()) {
            if( libraryname == (*it)->GetLibraryName() ) {
                lib = *it;
                lib->AddIKName(ikname);
                break;
            }
        }
        if( !lib ) {
            lib.reset(new IKLibrary());
            if( !lib->Init(ikname, libraryname) ) {
                return boost::shared_ptr<IKLibrary>();
            }
            GetLibraries().push_back(lib);
        }
        return lib;
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
            string cmdhas = str(boost::format("openrave.py --database inversekinematics --gethas --robot=%s --manipname=%s --iktype=%s")%probot->GetXMLFilename()%probot->GetActiveManipulator()->GetName()%striktype);
            FILE* pipe = popen(cmdhas.c_str(), "r");
            string hasik;
            {
                boost::iostreams::stream_buffer<boost::iostreams::file_descriptor_source> fpstream(fileno(pipe));
                std::istream in(&fpstream);
                std::getline(in, hasik);
            }
            pclose(pipe);
            if( hasik != "1" ) {
                RAVELOG_INFO(str(boost::format("Generating inverse kinematics for manip %s:%s\n")%probot->GetName()%probot->GetActiveManipulator()->GetName()));
                string cmdgen = str(boost::format("openrave.py --database inversekinematics --robot=%s --manipname=%s --iktype=%s")%probot->GetXMLFilename()%probot->GetActiveManipulator()->GetName()%striktype);
                FILE* pipe = popen(cmdgen.c_str(), "r");
                int generateexit = pclose(pipe);
                RAVELOG_INFO("generate exit: %d\n",generateexit);
            }
        }
        
        string cmdfilename = str(boost::format("openrave.py --database inversekinematics --getfilename --robot=%s --manipname=%s --iktype=%s")%probot->GetXMLFilename()%probot->GetActiveManipulator()->GetName()%striktype);
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

        boost::shared_ptr<IKLibrary> lib = _AddIkLibrary(ikfastname,ikfilename);
        if( !lib ) {
            return false;
        }
        if( lib->GetIKType() != niktype ) {
            return false;
        }
        IkSolverBasePtr iksolver = lib->CreateSolver(GetEnv(), 0.1);
        if( !iksolver ) {
            return false;
        }
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

    bool IKtest(ostream& sout, istream& sinput)
    {
        RAVELOG_DEBUGA("Starting IKtest...\n");
        vector<dReal> varmjointvals, values;

        TransformMatrix handTm;
        bool bCheckCollision = true, bInitialized=false;
        RobotBasePtr robot;
        RobotBase::ManipulatorConstPtr pmanip;
        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "trans" ) {
                RAVELOG_WARN("IKtest: trans parameter has been deprecated, switch to matrix/matrices/poses");
                sinput >> handTm.trans.x >> handTm.trans.y >> handTm.trans.z;
                bInitialized = true;
            }
            else if( cmd == "rot" ) {
                RAVELOG_WARN("IKtest: rot parameter has been deprecated, switch to matrix/matrices/poses");
                sinput >> handTm.m[0] >> handTm.m[4] >> handTm.m[8]
                       >> handTm.m[1] >> handTm.m[5] >> handTm.m[9]
                       >> handTm.m[2] >> handTm.m[6] >> handTm.m[10];
                bInitialized = true;
            }
            else if( cmd == "matrix" ) {
                sinput >> handTm;
                bInitialized = true;
            }
            else if( cmd == "armjoints" ) {
                varmjointvals.resize(pmanip->GetArmIndices().size());
                FOREACH(it, varmjointvals)
                    sinput >> *it;
            }
            else if( cmd == "nocol" ) {
                bCheckCollision = false;
            }
            else if( cmd == "robot" ) {
                string name;
                sinput >> name;
                robot = GetEnv()->GetRobot(name);
                pmanip = robot->GetActiveManipulator();
            }
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if( !robot ) {
            return false;
        }
        RobotBase::RobotStateSaver saver(robot);

        if( !bInitialized ) {
            handTm = pmanip->GetEndEffectorTransform();
        }
        Transform handTr(handTm);
    
        robot->GetDOFValues(values);

        for(size_t i = 0; i < varmjointvals.size(); i++)
            values[pmanip->GetArmIndices()[i]] = varmjointvals[i];

        robot->SetJointValues(values);

        vector<dReal> q1;
    
        if( !pmanip->FindIKSolution(handTr, q1, bCheckCollision) ) {
            RAVELOG_WARNA("No IK solution found\n");
            return false;
        }
    
        stringstream s2;
        s2 << "ik sol: ";
        FOREACH(it, q1) {
            s2 << *it << " ";
            sout << *it << " ";
        }
        s2 << endl;
        RAVELOG_DEBUGA(s2.str());
        return true;
    }

    bool DebugIKFindSolution(RobotBase::ManipulatorPtr pmanip, const Transform& twrist,
                             vector<dReal>& viksolution, bool bEnvCollision, vector<dReal>& parameters, int paramindex)
    {
        for(dReal f = 0; f <= 1; f += 0.001f) {
            parameters[paramindex] = f;
            if( paramindex > 0 ) {
                if( DebugIKFindSolution(pmanip, twrist, viksolution, bEnvCollision, parameters, paramindex-1) )
                    return true;
            }
            else {
                if( pmanip->FindIKSolution(twrist, parameters, viksolution, bEnvCollision) )
                    return true;
            }
        }

        return false;
    }
        
    void DebugIKFindSolutions(RobotBase::ManipulatorPtr pmanip, const Transform& twrist,
                              vector< vector<dReal> >& viksolutions, bool bEnvCollision,
                              vector<dReal>& parameters, int paramindex)
    {
        for(dReal f = 0; f <= 1; f += 0.001f) {
            parameters[paramindex] = f;
            if( paramindex > 0 ) {
                DebugIKFindSolutions(pmanip, twrist, viksolutions, bEnvCollision, parameters, paramindex-1);
            }
            else {
                vector< vector<dReal> > vtempsol;
                if( pmanip->FindIKSolutions(twrist, parameters, vtempsol, bEnvCollision) ) {
                    viksolutions.insert(viksolutions.end(), vtempsol.begin(), vtempsol.end());
                }
            }
        }
    }

    bool DebugIK(ostream& sout, istream& sinput)
    {
        int num_itrs = 10000;
        stringstream s;
        fstream fsfile;

        string readfilename, genfilename;
        bool bReadFile = false;
        bool bGenFile = false;
        dReal frotweight = 0.4f, ftransweight = 1.0f;

        RobotBasePtr robot;
        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "readfile" ) {
                sinput >> readfilename;
                bReadFile = true;
            }
            else if( cmd == "genfile" ) {
                sinput >> genfilename;
                bGenFile = true;
            }
            else if( cmd == "numtests" )
                sinput >> num_itrs;
            else if( cmd == "rotonly" ) {
                ftransweight = 0;
            }
            else if( cmd == "robot" ) {
                string name;
                sinput >> name;
                robot = GetEnv()->GetRobot(name);
            }
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
        vector<dReal> vjoints(pmanip->GetArmIndices().size(),0), vrand(pmanip->GetArmIndices().size(),0);
        vector<dReal> vlowerlimit, vupperlimit, viksolution;
        vector< vector<dReal> > viksolutions;

        RAVELOG_DEBUGA("Starting DebugIK... iter=%d\n", num_itrs);
    
        robot->SetActiveDOFs(pmanip->GetArmIndices());
        robot->GetActiveDOFLimits(vlowerlimit, vupperlimit);
    
        if(bGenFile) {
            fsfile.open(genfilename.c_str(),ios_base::out);
            fsfile << num_itrs <<endl;
        }
        if(bReadFile) {
            fsfile.open(readfilename.c_str(),ios_base::in);
            if(!fsfile.is_open())
                {
                    RAVELOG_ERRORA("IKFastProblem::DebugIK - Error: Cannot open specified file.\n");
                    return false;
                }

            fsfile >> num_itrs;
        }

        vector<dReal> vfreeparams(pmanip->GetNumFreeParameters()) ,vfreeparams2(pmanip->GetNumFreeParameters());

        Transform twrist, twrist_out;
        int i = 0;
        int success = 0;
        while(i < num_itrs) {
            if(bReadFile) {
                FOREACH(it, vjoints)
                    fsfile >> *it;
            
                fsfile >> twrist;
            }
            else {
                if( i == 0 ) {
                    // test the all 0s case (it is so common to get this imperfect)
                    for(int j = 0; j < (int)vjoints.size(); j++) {
                        vjoints[j] = 0;
                    }
                }
                else {
                    for(int j = 0; j < (int)vjoints.size(); j++) {
                        if( RaveRandomFloat() > 0.2f ) {
                            vjoints[j] = vlowerlimit[j] + (vupperlimit[j]-vlowerlimit[j])*RaveRandomFloat();
                        }
                        else {
                            switch(RaveRandomInt()%3) {
                                case 0: vjoints[j] = -PI*0.5; break;
                                case 2: vjoints[j] = PI*0.5; break;
                                default:
                                    vjoints[j] = 0;
                            }
                        }
                    }
                }
            }

            robot->SetActiveDOFValues(vjoints,true);

            if(GetEnv()->CheckCollision(KinBodyConstPtr(robot)) ) {
                RAVELOG_VERBOSEA("robot in collision\n");
                continue;
            }
            if( robot->CheckSelfCollision()) {
                RAVELOG_VERBOSEA("robot in self-collision\n");
                continue;
            }

            GetEnv()->UpdatePublishedBodies();
            RAVELOG_DEBUGA("iteration %d\n",i);
            twrist = pmanip->GetEndEffectorTransform();

            if(bGenFile) {
                FOREACH(it, vjoints)
                    fsfile << *it << " ";  
                fsfile << twrist << endl;
            }

            // find a random config
            while(1) {
                for(int j = 0; j < (int)vrand.size(); j++)
                    vrand[j] = vlowerlimit[j] + (vupperlimit[j]-vlowerlimit[j])*RaveRandomFloat();
        
                robot->SetActiveDOFValues(vrand, true);
                if(!GetEnv()->CheckCollision(KinBodyConstPtr(robot)) && !robot->CheckSelfCollision())
                    break;
            }
            
            if( !pmanip->FindIKSolution(twrist, viksolution, true) ) {    
                s.str("");
                s << "FindIKSolution: No ik solution found, i = " << i << endl << "Joint Val: ";
                FOREACH(it, vjoints)
                    s << *it << " ";
                s << endl << "Transform: " << twrist << endl << endl;
                RAVELOG_WARNA(s.str());
                ++i;
                continue;
            }

            robot->SetActiveDOFValues(viksolution, true);
            twrist_out = pmanip->GetEndEffectorTransform();
        
            if(_TransformDistance2(twrist, twrist_out, frotweight, ftransweight) > 0.05f*0.05f) {
                s.str("");
                s << "FindIKSolution: Incorrect IK, i = " << i <<" error: " << RaveSqrt(_TransformDistance2(twrist, twrist_out, frotweight, ftransweight)) << endl
                  << "Original Joint Val: ";
                FOREACH(it, vjoints)
                    s << *it << " ";
                s << endl << "Returned Joint Val: ";
                FOREACH(it, viksolution)
                    s << *it << " ";
                s << endl << "Transform in: " << twrist << endl;
                s << "Transform out: " << twrist_out << endl << endl;
                RAVELOG_WARNA(s.str());
                ++i;
                continue;
            }
        
            // test the multiple solution function
            robot->SetActiveDOFValues(vrand, true);
            if( !pmanip->FindIKSolutions(twrist, viksolutions, true) ) {
                s.str("");
                s << "FindIKSolutions: No ik solution found for, i = " << i << endl << "Joint Val: ";
                FOREACH(it, vjoints)
                    s << *it << " ";
                s << endl << "Transform: " << twrist << endl << endl;
                RAVELOG_WARNA(s.str());
                ++i;
                continue;
            }

            bool bfail = false;
            FOREACH(itsol, viksolutions) {
                robot->SetActiveDOFValues(*itsol, true);
                twrist_out = pmanip->GetEndEffectorTransform();
                if(_TransformDistance2(twrist, twrist_out, frotweight, ftransweight) > 0.05f*0.05f) {
                    s.str("");
                    s << "FindIKSolutions: Incorrect IK, i = " << i << " error: " << RaveSqrt(_TransformDistance2(twrist, twrist_out, frotweight, ftransweight)) << endl
                      << "Original Joint Val: ";
                    FOREACH(it, vjoints)
                        s << *it << " ";
                    s << endl << "Returned Joint Val: ";
                    FOREACH(it, *itsol)
                        s << *it << " ";
                    s << endl << "Transform in: " << twrist << endl;
                    s << "Transform out: " << twrist_out << endl << endl;
                    RAVELOG_WARNA(s.str());
                    bfail = true;
                    break;
                }
            }

            if( bfail ) {
                ++i;
                continue;
            }

            if( pmanip->GetNumFreeParameters() == 0 ) {
                success++;
                i++;
                continue;
            }

            // test with the free parameters
            robot->SetActiveDOFValues(vrand, true);
            if( !DebugIKFindSolution(pmanip, twrist, viksolution, true, vfreeparams, vfreeparams.size()-1) ) {
                s.str("");
                s << "FindIKSolution (freeparams: ";
                FOREACH(itfree,vfreeparams) {
                    s << *itfree << " ";
                }
                s << "): No ik solution found, i = " << i << endl << "Joint Val: ";
                for(size_t j = 0; j < vjoints.size(); j++)
                    s << vjoints[j] << " ";
                s << endl << "Transform: " << twrist << endl << endl;
                RAVELOG_WARNA(s.str());
                ++i;
                continue;
            }

            robot->SetActiveDOFValues(viksolution, true);
            twrist_out = pmanip->GetEndEffectorTransform();
        
            if(_TransformDistance2(twrist, twrist_out, frotweight, ftransweight) > 0.05f*0.05f) {
                s.str("");
                s << "FindIKSolution (freeparams): Incorrect IK, i = " << i << " error: " << RaveSqrt(_TransformDistance2(twrist, twrist_out, frotweight, ftransweight)) << endl
                  << "freeparams: ";
                FOREACH(it, vfreeparams)
                    s << *it << " ";
                s << endl << "Original Joint Val: ";
                FOREACH(it, vjoints)
                    s << *it << " ";
                s << endl << "Returned Joint Val: ";
                FOREACH(it, viksolution)
                    s << *it << " ";
                s << endl << "Transform in: " << twrist << endl;
                s << "Transform out: " << twrist_out << endl << endl;
                RAVELOG_WARNA(s.str());
                ++i;
                continue;
            }

            if( !pmanip->GetFreeParameters(vfreeparams2) ) {
                RAVELOG_ERRORA("failed to get free parameters\n");
                ++i;
                continue;
            }

            // make sure they are the same
            for(int j = 0; j < pmanip->GetNumFreeParameters(); ++j) {
                if( fabsf(vfreeparams[j]-vfreeparams2[j]) > 0.01f ) {
                    RAVELOG_WARNA("free params %d not equal: %f!=%f\n", j, vfreeparams[j], vfreeparams2[j]);
                    pmanip->GetFreeParameters(vfreeparams2);
                    ++i;
                    continue;
                }
            }
        
            // test the multiple solution function
            robot->SetActiveDOFValues(vrand, true);
            viksolution.resize(0);
            DebugIKFindSolutions(pmanip, twrist, viksolutions, true, vfreeparams, vfreeparams.size()-1);

            if( viksolutions.size() == 0 ) {
                s.str("");
                s << "FindIKSolutions (freeparams): No ik solution found for, i = " << i << endl << "Joint Val: ";
                for(size_t j = 0; j < vjoints.size(); j++)
                    s << vjoints[j] << " ";
                s << endl << "Transform: " << twrist << endl << endl;
                RAVELOG_WARNA(s.str());
                ++i;
                continue;
            }

            bfail = false;
            FOREACH(itsol, viksolutions) {
                robot->SetActiveDOFValues(*itsol, true);
                twrist_out = pmanip->GetEndEffectorTransform();
                if(_TransformDistance2(twrist, twrist_out, frotweight, ftransweight) > 0.05f*0.05f) {
                    s.str("");
                    s << "FindIKSolutions (freeparams): Incorrect IK, i = " << i <<" error: " << RaveSqrt(_TransformDistance2(twrist, twrist_out, frotweight, ftransweight)) << endl
                      << "Original Joint Val: ";
                    FOREACH(it, vjoints)
                        s << *it << " ";
                    s << endl << "Returned Joint Val: ";
                    FOREACH(it, *itsol)
                        s << *it << " ";
                    s << endl << "Transform in: " << twrist << endl;
                    s << "Transform out: " << twrist_out << endl << endl;
                    RAVELOG_WARNA(s.str());
                    bfail = true;
                    break;
                }
            }

            if( bfail ) {
                ++i;
                continue;
            }

            success++;
            i++;
        }

        RAVELOG_INFOA("DebugIK done, success rate %f.\n", (float)success/(float)num_itrs);
        sout << (float)success/(float)num_itrs;
        return true;
    }

    static list< boost::shared_ptr<IKLibrary> >& GetLibraries()
    {
        static list< boost::shared_ptr<IKLibrary> > s_vStaticLibraries;
        return s_vStaticLibraries;
    }

    static boost::mutex& GetLibraryMutex()
    {
        static boost::mutex s_LibraryMutex;
        return s_LibraryMutex;
    }

    static IkSolverBasePtr CreateIkSolver(const string& _name, dReal freeinc, EnvironmentBasePtr penv)
    {
        string name; name.resize(_name.size());
        std::transform(_name.begin(), _name.end(), name.begin(), ::tolower);
        /// start from the newer libraries
        boost::mutex::scoped_lock lock(GetLibraryMutex());
        for(list< boost::shared_ptr<IKLibrary> >::reverse_iterator itlib = GetLibraries().rbegin(); itlib != GetLibraries().rend(); ++itlib) {
            FOREACHC(itikname,(*itlib)->GetIKNames()) {
                if( name == *itikname ) {
                    return (*itlib)->CreateSolver(penv,freeinc);
                }
            }
        }
        return IkSolverBasePtr();
    }

private:
    inline static dReal _TransformDistance2(const Transform& t1, const Transform& t2, dReal frotweight=1, dReal ftransweight=1)
    {
        dReal facos = RaveAcos(min(dReal(1),RaveFabs(dot4(t1.rot,t2.rot))));
        return (t1.trans-t2.trans).lengthsqr3() + frotweight*facos*facos;
    }
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(IKSolutionFloat)
BOOST_TYPEOF_REGISTER_TYPE(IKSolutionDouble)
BOOST_TYPEOF_REGISTER_TYPE(IKFastProblem)
#endif

#endif
