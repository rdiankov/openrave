#ifndef OPENRAVE_IKFAST_PROBLEM
#define OPENRAVE_IKFAST_PROBLEM

#include "plugindefs.h"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#ifdef Boost_IOSTREAMS_FOUND
#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/version.hpp>
#if BOOST_VERSION >= 104400
#define FILE_DESCRIPTOR_FLAG boost::iostreams::never_close_handle
#else
#define FILE_DESCRIPTOR_FLAG false
#endif
#endif

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define MYPOPEN _popen
#define MYPCLOSE _pclose
#else
#include <dlfcn.h>
#include <sys/types.h>
#include <dirent.h>
#define MYPOPEN popen
#define MYPCLOSE pclose
#endif

#ifdef HAVE_BOOST_FILESYSTEM
#include <boost/filesystem/operations.hpp>
#endif

#define DECLFNPTR(name, paramlist) (*name)paramlist

class IKFastProblem : public ModuleBase
{
    typedef int DECLFNPTR (getNumFreeParametersFn, ());
    typedef int* DECLFNPTR (getFreeParametersFn, ());
    typedef int DECLFNPTR (getNumJointsFn, ());
    typedef const char* DECLFNPTR (getKinematicsHashFn, ());
    typedef int DECLFNPTR (getIKRealSizeFn, ());
    typedef int DECLFNPTR (getIKTypeFn, ());
    static int getDefaultIKType() {
        return IKP_Transform6D;
    }

    class IKLibrary : public boost::enable_shared_from_this<IKLibrary>
    {
public:
        IKLibrary() : getKinematicsHash(NULL), plib(NULL) {
        }
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
                RAVELOG_WARN("failed to load library %s\n", _libraryname.c_str());
                return false;
            }

            getNumFreeParameters = (getNumFreeParametersFn)SysLoadSym(plib, "getNumFreeParameters");
            if( getNumFreeParameters == NULL ) {
                RAVELOG_WARN("failed to find getNumFreeParameters in %s\n", _libraryname.c_str());
                return false;
            }
            getFreeParameters = (getFreeParametersFn)SysLoadSym(plib, "getFreeParameters");
            if( getFreeParameters == NULL ) {
                RAVELOG_WARN("failed to find getFreeParameters in %s\n", _libraryname.c_str());
                return false;
            }
            getNumJoints = (getNumJointsFn)SysLoadSym(plib, "getNumJoints");
            if( getNumJoints == NULL ) {
                RAVELOG_WARN("failed to find getNumJoints in %s\n", _libraryname.c_str());
                return false;
            }
            getIKRealSize = (getIKRealSizeFn)SysLoadSym(plib, "getIKRealSize");
            if( getIKRealSize == NULL ) {
                RAVELOG_WARN("failed to find getIKRealSize in %s\n", _libraryname.c_str());
                return false;
            }
            getIKType = (getIKTypeFn)SysLoadSym(plib, "getIKType");
            if( getIKType == NULL ) {
                RAVELOG_WARN("failed to find getIKType in %s, setting to 6D transform\n", _libraryname.c_str());
                getIKType = getDefaultIKType;
                //return false;
            }
            getKinematicsHash = (getKinematicsHashFn)SysLoadSym(plib, "getKinematicsHash");
            if( getKinematicsHash == NULL ) {
                RAVELOG_WARN("failed to find getKinematicsHash in %s, will not be able to validate inverse kinematics structure\n", _libraryname.c_str());
            }

            ikfn = SysLoadSym(plib, "ik");
            if( ikfn == NULL ) {
                RAVELOG_WARN("failed to find ik in %s\n", _libraryname.c_str());
                return false;
            }
            fkfn = SysLoadSym(plib, "fk");

            vfree.resize(getNumFreeParameters());
            for(size_t i = 0; i < vfree.size(); ++i) {
                vfree[i] = getFreeParameters()[i];
            }
            return true;
        }

        IkSolverBasePtr CreateSolver(EnvironmentBasePtr penv, const vector<dReal>& vfreeinc)
        {
            string kinematicshash;
            if( getKinematicsHash != NULL ) {
                kinematicshash =getKinematicsHash();
            }
            if( getIKRealSize() == 4 ) {
                return IkSolverBasePtr(new IkFastSolver<float,IKSolutionFloat >((IkFastSolver<float,IKSolutionFloat >::IkFn)ikfn,vfree,vfreeinc,getNumJoints(),(IkParameterizationType)getIKType(),shared_from_this(),kinematicshash, penv));
            }
            else if( getIKRealSize() == 8 ) {
                return IkSolverBasePtr(new IkFastSolver<double,IKSolutionDouble >((IkFastSolver<double,IKSolutionDouble >::IkFn)ikfn,vfree,vfreeinc,getNumJoints(),(IkParameterizationType)getIKType(),shared_from_this(),kinematicshash, penv));
            }
            throw openrave_exception("bad real size");
        }

        const vector<string>& GetIKNames() const {
            return _viknames;
        }
        void AddIKName(const string& ikname) {
            _viknames.push_back(ikname);
        }
        const string& GetLibraryName() const {
            return _libraryname;
        }
        int GetIKType() {
            return getIKType();
        }

        getNumFreeParametersFn getNumFreeParameters;
        getFreeParametersFn getFreeParameters;
        getNumJointsFn getNumJoints;
        getKinematicsHashFn getKinematicsHash;
        getIKRealSizeFn getIKRealSize;
        getIKTypeFn getIKType;
        void* ikfn, *fkfn;

private:
        void* SysLoadLibrary(const char* lib)
        {
#ifdef _WIN32
            void* plib = LoadLibraryA(lib);
            if( plib == NULL ) {
                RAVELOG_WARN("Failed to load %s\n", lib);
            }
#else
            void* plib = dlopen(lib, RTLD_NOW);
            if( plib == NULL ) {
                RAVELOG_WARN("%s\n", dlerror());
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

    inline boost::shared_ptr<IKFastProblem> shared_problem() {
        return boost::static_pointer_cast<IKFastProblem>(shared_from_this());
    }
    inline boost::shared_ptr<IKFastProblem const> shared_problem_const() const {
        return boost::static_pointer_cast<IKFastProblem const>(shared_from_this());
    }

public:
    IKFastProblem(EnvironmentBasePtr penv) : ModuleBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\nAllows dynamic loading and registering of ikfast shared objects to openrave plugins.\nAlso contains several test routines for inverse kinematics.";
        RegisterCommand("AddIkLibrary",boost::bind(&IKFastProblem::AddIkLibrary,this,_1,_2),
                        "Dynamically adds an ik solver to openrave by loading a shared object (based on ikfast code generation).\n"
                        "Usage::\n\n  AddIkLibrary iksolvername iklibrarypath\n\n"
                        "return the type of inverse kinematics solver (IkParamterization::Type)");
#ifdef Boost_IOSTREAMS_FOUND
        RegisterCommand("LoadIKFastSolver",boost::bind(&IKFastProblem::LoadIKFastSolver,this,_1,_2),
                        "Dynamically calls the inversekinematics.py script to generate an ik solver for a robot, or to load an existing one\n"
                        "Usage::\n\n  LoadIKFastSolver robotname iktype_id [free increment]\n\n"
                        "return nothing, but does call the SetIKSolver for the robot");
#endif
        RegisterCommand("PerfTiming",boost::bind(&IKFastProblem::PerfTiming,this,_1,_2),
                        "Times the ik call of a given library.\n"
                        "Usage::\n\n  PerfTiming [options] iklibrarypath\n\n"
                        "return the set of time measurements made in nano-seconds");
        RegisterCommand("IKTest",boost::bind(&IKFastProblem::IKtest,this,_1,_2),
                        "Tests for an IK solution if active manipulation has an IK solver attached");
        RegisterCommand("DebugIK",boost::bind(&IKFastProblem::DebugIK,this,_1,_2),
                        "Function used for debugging and testing an IK solver. Input parameters are:\n\n\
* string readfile - file containing joint values to read, starts with number of entries.\n\n\
* int numtests - if file not specified, number of random tests to perform (defualt is 1000).\n\n\
* float sampledegeneratecases - probability in [0,1] specifies the probability of sampling joint values on [-pi/2,0,pi/2] (default is 0.2).\n\n\
* int selfcollision - if true, will check IK only for non-self colliding positions of the robot (default is 0).\n\n\
* string robot - name of the robot to test. the active manipulator of the roobt is used.\n\n");
    }

    virtual ~IKFastProblem() {
    }

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
        if (!getline(sinput, libraryname) ) {
            return false;
        }
        boost::trim(libraryname);
        if( !sinput ||( libraryname.size() == 0) ||( ikname.size() == 0) ) {
            RAVELOG_DEBUG("bad input\n");
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
        FOREACH(it, *GetLibraries()) {
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
            GetLibraries()->push_back(lib);
        }
        return lib;
    }

#ifdef Boost_IOSTREAMS_FOUND
    bool LoadIKFastSolver(ostream& sout, istream& sinput)
    {
        EnvironmentMutex::scoped_lock envlock(GetEnv()->GetMutex());
        string robotname;
        string striktype;
        bool bForceIK = false;
        sinput >> robotname;
        sinput >> striktype;
        if( !sinput ) {
            return false;
        }
        sinput >> bForceIK;     // optional
        RobotBasePtr probot = GetEnv()->GetRobot(robotname);
        if( !probot || !probot->GetActiveManipulator() ) {
            return false;
        }
        RobotBase::ManipulatorPtr pmanip = probot->GetActiveManipulator();
        IkParameterizationType niktype = IKP_None;
        try {
            niktype = static_cast<IkParameterizationType>(boost::lexical_cast<int>(striktype));
            striktype = RaveGetIkParameterizationMap().find(niktype)->second;
        }
        catch(const boost::bad_lexical_cast&) {
            // striktype is already correct, so check that it exists in RaveGetIkParameterizationMap
            std::transform(striktype.begin(), striktype.end(), striktype.begin(), ::tolower);
            FOREACHC(it,RaveGetIkParameterizationMap()) {
                string mapiktype = it->second;
                std::transform(mapiktype.begin(), mapiktype.end(), mapiktype.begin(), ::tolower);
                if( mapiktype == striktype ) {
                    niktype = it->first;
                    break;
                }
            }
            if(niktype == IKP_None) {
                throw openrave_exception(str(boost::format("could not find iktype %s")%striktype));
            }
        }

        {
            string hasik;
            string cmdhas = str(boost::format("openrave.py --database inversekinematics --gethas --robot=\"%s\" --manipname=%s --iktype=%s")%probot->GetURI()%pmanip->GetName()%striktype);
            FILE* pipe = MYPOPEN(cmdhas.c_str(), "r");
            {
                boost::iostreams::stream_buffer<boost::iostreams::file_descriptor_source> fpstream(fileno(pipe),FILE_DESCRIPTOR_FLAG);
                std::istream in(&fpstream);
                std::getline(in, hasik);
            }
            int generateexit = MYPCLOSE(pipe);
            if( generateexit != 0 ) {
                Sleep(100);
                RAVELOG_DEBUG("failed to close pipe\n");
            }
            boost::trim(hasik);
            if( hasik != "1" ) {
                RAVELOG_INFO(str(boost::format("Generating inverse kinematics for manip %s:%s, will take several minutes...\n")%probot->GetName()%pmanip->GetName()));
                string cmdgen = str(boost::format("openrave.py --database inversekinematics --usecached --robot=\"%s\" --manipname=%s --iktype=%s")%probot->GetURI()%pmanip->GetName()%striktype);
                // use raw system call, popen causes weird crash in the inversekinematics compiler
                int generateexit = system(cmdgen.c_str());
                //FILE* pipe = MYPOPEN(cmdgen.c_str(), "r");
                //int generateexit = MYPCLOSE(pipe);
                if( generateexit != 0 ) {
                    Sleep(100);
                    RAVELOG_DEBUG("failed to close pipe\n");
                }
            }
        }

        string cmdfilename = str(boost::format("openrave.py --database inversekinematics --getfilename --robot=\"%s\" --manipname=%s --iktype=%s")%probot->GetURI()%pmanip->GetName()%striktype);
        RAVELOG_INFO("executing shell command:\n%s\n",cmdfilename.c_str());
        string ikfilename;
        FILE* pipe = MYPOPEN(cmdfilename.c_str(), "r");
        {
            boost::iostreams::stream_buffer<boost::iostreams::file_descriptor_source> fpstream(fileno(pipe),FILE_DESCRIPTOR_FLAG);
            std::istream in(&fpstream);
            std::getline(in, ikfilename);
            if( !in ) {
                RAVELOG_INFO("filed to get line: %s?!\n",ikfilename.c_str());
                MYPCLOSE(pipe);
                return false;
            }
        }
        int generateexit = MYPCLOSE(pipe);
        if( generateexit != 0 ) {
            Sleep(100);
            RAVELOG_DEBUG("failed to close pipe\n");
        }

        boost::trim(ikfilename);
        string ikfastname = str(boost::format("ikfast.%s.%s")%probot->GetRobotStructureHash()%pmanip->GetName());
        boost::shared_ptr<IKLibrary> lib = _AddIkLibrary(ikfastname,ikfilename);
        bool bsuccess = true;
        if( !lib ) {
            bsuccess = false;
        }
        else {
            if( lib->GetIKType() != (int)niktype ) {
                bsuccess = false;
            }
            else {
                IkSolverBasePtr iksolver = lib->CreateSolver(GetEnv(), vector<dReal>());
                if( !iksolver ) {
                    bsuccess = false;
                }
                else {
                    bsuccess = pmanip->SetIkSolver(iksolver);
                }
            }
        }
        // if not forcing the ik, then return true as long as a valid ik solver is set
        if( bForceIK && !bsuccess ) {
            return false;
        }
        return !!pmanip->GetIkSolver() && pmanip->GetIkSolver()->Supports(niktype);
    }
#endif

    bool PerfTiming(ostream& sout, istream& sinput)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        string cmd, libraryname;
        int num=1000;
        dReal maxtime = 1200;
        while(!sinput.eof()) {
            istream::streampos pos = sinput.tellg();
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "num" ) {
                sinput >> num;
            }
            else if( cmd == "maxtime" ) {
                sinput >> maxtime;
            }
            else {
                sinput.clear();     // have to clear eof bit
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
            return _PerfTiming<IKSolutionFloat>(sout,lib,num, maxtime);
        }
        else if( lib->getIKRealSize() == 8 ) {
            return _PerfTiming<IKSolutionDouble>(sout,lib,num, maxtime);
        }
        else {
            throw openrave_exception("bad real size");
        }
        return true;
    }

    template<typename T> bool _PerfTiming(ostream& sout, boost::shared_ptr<IKLibrary> lib, int num, dReal maxtime)
    {
        BOOST_ASSERT(lib->getIKRealSize()==sizeof(typename T::IKReal));
        typename IkFastSolver<typename T::IKReal,T>::IkFn ikfn = (typename IkFastSolver<typename T::IKReal,T>::IkFn)lib->ikfn;
        typename IkFastSolver<typename T::IKReal,T>::FkFn fkfn = (typename IkFastSolver<typename T::IKReal,T>::FkFn)lib->fkfn;
        if( !ikfn || !fkfn ) {
            return false;
        }
        vector<uint64_t> vtimes(num);
        vector<T> vsolutions; vsolutions.reserve(32);
        vector<typename T::IKReal> vjoints(lib->getNumJoints()), vfree(lib->getNumFreeParameters());
        typename T::IKReal eerot[9],eetrans[3];
        uint32_t runstarttimems = GetMilliTime();
        uint32_t runmaxtimems = (uint32_t)(1000*maxtime);
        size_t i = 0;
        for(i = 0; i < vtimes.size(); ++i) {
            // don't want to slow down the tests too much with polling
            if(( (i%100) == 0) &&( (GetMilliTime() - runstarttimems) > runmaxtimems) ) {
                break;
            }
            for(size_t j = 0; j < vjoints.size(); ++j) {
                vjoints[j] = RaveRandomDouble()*2*PI;
            }
            for(size_t j = 0; j < vfree.size(); ++j) {
                vfree[j] = vjoints[lib->getFreeParameters()[j]];
            }
            fkfn(&vjoints[0],eetrans,eerot);
            vsolutions.resize(0);
            uint64_t numtoaverage=10;
            uint64_t starttime = GetNanoPerformanceTime();
            for(uint64_t j = 0; j < numtoaverage; ++j) {
                ikfn(eetrans,eerot,vfree.size() > 0 ? &vfree[0] : NULL,vsolutions);
            }
            vtimes[i] = (GetNanoPerformanceTime()-starttime)/numtoaverage;
        }
        while(i-- > 0) {
            sout << vtimes[i] << " ";
        }
        return true;
    }

    bool IKtest(ostream& sout, istream& sinput)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        RAVELOG_DEBUG("Starting IKtest...\n");
        vector<dReal> varmjointvals, values;
        bool bInitialized=false;
        int filteroptions = IKFO_CheckEnvCollisions;
        RobotBasePtr robot;
        RobotBase::ManipulatorConstPtr pmanip;
        IkParameterization ikparam;
        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
            if( cmd == "matrix" ) {
                TransformMatrix handTm;
                sinput >> handTm;
                ikparam.SetTransform6D(handTm);
                bInitialized = true;
            }
            else if( cmd == "ikparam" ) {
                sinput >> ikparam;
            }
            else if( cmd == "armjoints" ) {
                varmjointvals.resize(pmanip->GetArmIndices().size());
                FOREACH(it, varmjointvals) {
                    sinput >> *it;
                }
            }
            else if( cmd == "nocol" ) {
                filteroptions = 0;
            }
            else if( cmd == "robot" ) {
                string name;
                sinput >> name;
                robot = GetEnv()->GetRobot(name);
                pmanip = robot->GetActiveManipulator();
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if( !robot ) {
            return false;
        }
        RobotBase::RobotStateSaver saver(robot);

        if( !bInitialized ) {
            ikparam = pmanip->GetIkParameterization(IKP_Transform6D);
        }
        robot->GetDOFValues(values);

        for(size_t i = 0; i < varmjointvals.size(); i++) {
            values[pmanip->GetArmIndices()[i]] = varmjointvals[i];
        }
        robot->SetDOFValues(values);

        vector<dReal> q1;
        if( !pmanip->FindIKSolution(ikparam, q1, filteroptions) ) {
            RAVELOG_WARN("No IK solution found\n");
            return false;
        }

        stringstream s2;
        s2 << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
        s2 << "ik sol: ";
        FOREACH(it, q1) {
            s2 << *it << " ";
            sout << *it << " ";
        }
        s2 << endl;
        RAVELOG_DEBUG(s2.str());
        return true;
    }

    bool DebugIKFindSolution(RobotBase::ManipulatorPtr pmanip, const IkParameterization& twrist, std::vector<dReal>& viksolution, int filteroptions, std::vector<dReal>& parameters, int paramindex)
    {
        // ignore boundary cases since next to limits and can fail due to limit errosr
        for(dReal f = 0.0001; f <= 0.9999; f += 0.01f) {
            parameters[paramindex] = f;
            if( paramindex > 0 ) {
                if( DebugIKFindSolution(pmanip, twrist, viksolution, filteroptions, parameters, paramindex-1) ) {
                    return true;
                }
            }
            else {
                if( pmanip->FindIKSolution(twrist, parameters, viksolution, filteroptions) ) {
                    return true;
                }
            }
        }

        return false;
    }

    void DebugIKFindSolutions(RobotBase::ManipulatorPtr pmanip, const IkParameterization& twrist, vector< vector<dReal> >& viksolutions, int filteroptions, std::vector<dReal>& parameters, int paramindex)
    {
        // ignore boundary cases since next to limits and can fail due to limit errosr
        for(dReal f = 0.0001; f <= 0.9999; f += 0.01f) {
            parameters.at(paramindex) = f;
            if( paramindex > 0 ) {
                DebugIKFindSolutions(pmanip, twrist, viksolutions, filteroptions, parameters, paramindex-1);
            }
            else {
                vector< vector<dReal> > vtempsol;
                if( pmanip->FindIKSolutions(twrist, parameters, vtempsol, filteroptions) ) {
                    viksolutions.insert(viksolutions.end(), vtempsol.begin(), vtempsol.end());
                }
            }
        }
    }

    bool DebugIK(ostream& sout, istream& sinput)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());

        int num_itrs = 1000;
        stringstream s;
        s << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
        fstream fsfile;
        string readfilename;
        bool bReadFile = false, bTestSelfCollision = false;
        dReal sampledegeneratecases = 0.2f, fthreshold = 0.000001f;

        int filteroptions = IKFO_IgnoreJointLimits|IKFO_IgnoreSelfCollisions|IKFO_IgnoreCustomFilters;
        RobotBasePtr robot;
        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "readfile" ) {
                sinput >> readfilename;
                bReadFile = true;
            }
            else if( cmd == "numtests" ) {
                sinput >> num_itrs;
            }
            else if( cmd == "sampledegeneratecases" ) {
                sinput >> sampledegeneratecases;
            }
            else if( cmd == "selfcollision" ) {
                sinput >> bTestSelfCollision;
            }
            else if( cmd == "robot" ) {
                string name;
                sinput >> name;
                robot = GetEnv()->GetRobot(name);
            }
            else if( cmd == "threshold" ) {
                sinput >> fthreshold;
                fthreshold *= fthreshold;
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        RAVELOG_DEBUG("Starting DebugIK... max iterations=%d\n", num_itrs);
        RobotBase::RobotStateSaver saver(robot);
        robot->Enable(bTestSelfCollision);
        RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
        if( !pmanip->GetIkSolver() ) {
            RAVELOG_ERROR(str(boost::format("no ik solver set on manipulator %s")%pmanip->GetName()));
            return false;
        }

        // set the ik threshold to something big so wrong solutions can be returned
        stringstream soutik, sinputik; sinputik << "SetIkThreshold 1000";
        pmanip->GetIkSolver()->SendCommand(soutik,sinputik);

        vector<dReal> vrealsolution(pmanip->GetArmIndices().size(),0), vrand(pmanip->GetArmIndices().size(),0);
        vector<dReal> vlowerlimit, vupperlimit, viksolution;
        vector< vector<dReal> > viksolutions, viksolutions2;

        robot->SetActiveDOFs(pmanip->GetArmIndices());
        robot->GetActiveDOFLimits(vlowerlimit, vupperlimit);
        // shrink the limits to prevent solutions close to limits from returning errors
        for(size_t i = 0; i < vlowerlimit.size(); ++i) {
            dReal newlower = vlowerlimit[i]*0.9999+0.0001*vupperlimit[i];
            dReal newupper = vlowerlimit[i]*0.0001+0.9999*vupperlimit[i];
            vlowerlimit[i] = newlower;
            vupperlimit[i] = newupper;
        }

        if(bReadFile) {
            fsfile.open(readfilename.c_str(),ios_base::in);
            if(!fsfile.is_open()) {
                RAVELOG_ERROR("IKFastProblem::DebugIK - Error: Cannot open specified file.\n");
                return false;
            }
            fsfile >> num_itrs;
        }

        RaveInitRandomGeneration(GetMilliTime());     // have to seed a new number

        IkParameterization twrist, twrist_out;
        vector<dReal> vfreeparameters_real, vfreeparameters, vfreeparameters_out;
        boost::array<vector<pair<IkParameterization, vector<dReal> > >, 3> vsolutionresults;
        vector<pair<IkParameterization, vector<dReal> > >& vwrongsolutions = vsolutionresults[0];     // wrong solution is returned
        vector<pair<IkParameterization, vector<dReal> > >& vnosolutions = vsolutionresults[1];     // no solution found
        vector<pair<IkParameterization, vector<dReal> > >& vnofullsolutions = vsolutionresults[2];     // solution returned, but not all of them
        int success=0;
        int nTotalIterations = 0;

        FOREACHC(itiktype, RaveGetIkParameterizationMap()) {
            if( !pmanip->GetIkSolver()->Supports(itiktype->first) ) {
                continue;
            }
            nTotalIterations += num_itrs;
            int i = 0;
            while(i < num_itrs) {
                if(bReadFile) {
                    FOREACH(it, vrealsolution) {
                        fsfile >> *it;
                    }
                    if( !fsfile ) {
                        break;
                    }
                }
                else {
                    if( i == 0 ) {
                        // test the all 0s case (it is so common to get this imperfect)
                        for(int j = 0; j < (int)vrealsolution.size(); j++) {
                            vrealsolution[j] = 0;
                        }
                    }
                    else {
                        for(int j = 0; j < (int)vrealsolution.size(); j++) {
                            if( RaveRandomFloat() > sampledegeneratecases ) {
                                vrealsolution[j] = vlowerlimit[j] + (vupperlimit[j]-vlowerlimit[j])*RaveRandomFloat();
                            }
                            else {
                                switch(RaveRandomInt()%3) {
                                case 0: vrealsolution[j] = CLAMP_ON_RANGE(dReal(-PI*0.5),vlowerlimit[j],vupperlimit[j]); break;
                                case 2: vrealsolution[j] = CLAMP_ON_RANGE(dReal(PI*0.5),vlowerlimit[j],vupperlimit[j]); break;
                                default: vrealsolution[j] = CLAMP_ON_RANGE(dReal(0),vlowerlimit[j],vupperlimit[j]); break;
                                }
                            }
                        }
                    }
                }

                robot->SetActiveDOFValues(vrealsolution,false);
                if( itiktype->first == IKP_Lookat3D) {
                    twrist.SetLookat3D(Vector(RaveRandomFloat()-0.5,RaveRandomFloat()-0.5,RaveRandomFloat()-0.5)*10);
                    twrist = pmanip->GetIkParameterization(twrist);
                }
                else if( itiktype->first == IKP_TranslationLocalGlobal6D) {
                    twrist.SetTranslationLocalGlobal6D(Vector(RaveRandomFloat()-0.5,RaveRandomFloat()-0.5,RaveRandomFloat()-0.5),Vector());
                    twrist = pmanip->GetIkParameterization(twrist);
                }
                else {
                    twrist = pmanip->GetIkParameterization(itiktype->first);
                }

                if( !pmanip->GetIkSolver()->GetFreeParameters(vfreeparameters_real) ) {
                    RAVELOG_WARN("failed to get freeparameters");
                }
                if( bTestSelfCollision && robot->CheckSelfCollision()) {
                    RAVELOG_VERBOSE("robot in self-collision\n");
                    continue;
                }

                // have to start at a random config
                while(1) {
                    for(int j = 0; j < (int)vrand.size(); j++) {
                        vrand[j] = vlowerlimit[j] + (vupperlimit[j]-vlowerlimit[j])*RaveRandomFloat();
                    }
                    robot->SetActiveDOFValues(vrand, false);
                    if(!bTestSelfCollision || !robot->CheckSelfCollision()) {
                        break;
                    }
                }
                RAVELOG_DEBUG("iteration %d\n",i);
                if( !pmanip->GetIkSolver()->GetFreeParameters(vfreeparameters) ) {
                    RAVELOG_WARN("failed to get freeparameters");
                }
                vfreeparameters_out.resize(vfreeparameters.size());

                bool bsuccess = true;
                bool bnoiksolution = false;
                if( !pmanip->FindIKSolution(twrist, viksolution, filteroptions) ) {
                    if( !bnoiksolution ) {
                        vnosolutions.push_back(make_pair(twrist,vfreeparameters));
                        bnoiksolution = true;
                    }
                    bsuccess = false;
                    s.str("");
                    s << "FindIKSolution: No ik solution found, i = " << i << endl << "Joint Val: ";
                    FOREACH(it, vrealsolution) {
                        s << *it << " ";
                    }
                    s << endl << "Transform: " << twrist << endl;
                    s << "raw ik command: ";
                    GetIKFastCommand(s, pmanip->GetBase()->GetTransform().inverse()*twrist);
                    FOREACH(itfree,vfreeparameters) {
                        s << *itfree << " ";
                    }
                    s << endl << endl;
                    RAVELOG_WARN(s.str());
                }
                else {
                    robot->SetActiveDOFValues(viksolution,false);
                    twrist_out = pmanip->GetIkParameterization(twrist);
                    if( !pmanip->GetIkSolver()->GetFreeParameters(vfreeparameters_out) ) {
                        RAVELOG_WARN("failed to get freeparameters");
                    }
                    if( twrist.ComputeDistanceSqr(twrist_out) > fthreshold) {
                        vwrongsolutions.push_back(make_pair(twrist,vfreeparameters_out));
                        bsuccess = false;
                        s.str("");
                        s << "FindIKSolution: Incorrect IK, i = " << i <<" error: " << RaveSqrt(twrist.ComputeDistanceSqr(twrist_out)) << endl
                          << "Original Joint Val: ";
                        FOREACH(it, vrealsolution)
                        s << *it << " ";
                        s << endl << "Returned Joint Val: ";
                        FOREACH(it, viksolution)
                        s << *it << " ";
                        s << endl << "in: " << twrist << endl;
                        s << "out: " << twrist_out << endl;
                        s << "raw ik command: ";
                        GetIKFastCommand(s, pmanip->GetBase()->GetTransform().inverse()*twrist);
                        FOREACH(itfree,vfreeparameters_out) {
                            s << *itfree << " ";
                        }
                        s << endl << endl;
                        RAVELOG_ERROR(s.str());
                        ++i;
                        continue;
                    }
                }

                // test all possible solutions
                robot->SetActiveDOFValues(vrand, false);
                pmanip->FindIKSolutions(twrist, viksolutions, filteroptions);
                if( vfreeparameters_real.size() > 0 ) {
                    pmanip->FindIKSolutions(twrist, vfreeparameters_real, viksolutions2, filteroptions);
                    viksolutions.insert(viksolutions.end(),viksolutions2.begin(),viksolutions2.end());
                }
                if( viksolutions.size() == 0 ) {
                    FOREACH(itfree,vfreeparameters_out) {
                        *itfree = -1;
                    }
                    if( !bnoiksolution ) {
                        vnosolutions.push_back(make_pair(twrist,vfreeparameters_out));
                        bnoiksolution = true;
                    }
                    bsuccess = false;
                    s.str("");
                    s << "FindIKSolutions: No ik solution found for, i = " << i << endl << "Joint Val: ";
                    FOREACH(it, vrealsolution) {
                        s << *it << " ";
                    }
                    s << endl << "Transform: " << twrist << endl << endl;
                    RAVELOG_WARN(s.str());
                }
                else {
                    bool bfail = false;
                    bool bfoundinput = false;
                    FOREACH(itsol, viksolutions) {
                        robot->SetActiveDOFValues(*itsol, false);
                        twrist_out = pmanip->GetIkParameterization(twrist);
                        if( !pmanip->GetIkSolver()->GetFreeParameters(vfreeparameters_out) ) {
                            RAVELOG_WARN("failed to get freeparameters");
                        }
                        if(twrist.ComputeDistanceSqr(twrist_out) > fthreshold ) {
                            vwrongsolutions.push_back(make_pair(twrist,vfreeparameters_out));
                            s.str("");
                            s << "FindIKSolutions: Incorrect IK, i = " << i << " error: " << RaveSqrt(twrist.ComputeDistanceSqr(twrist_out)) << endl
                              << "Original Joint Val: ";
                            FOREACH(it, vrealsolution) {
                                s << *it << " ";
                            }
                            s << endl << "Returned Joint Val: ";
                            FOREACH(it, *itsol)
                            s << *it << " ";
                            s << endl << "in: " << twrist << endl;
                            s << "out: " << twrist_out << endl;
                            s << "raw ik command: ";
                            GetIKFastCommand(s, pmanip->GetBase()->GetTransform().inverse()*twrist);
                            FOREACH(itfree,vfreeparameters_out) {
                                s << *itfree << " ";
                            }
                            s << endl << endl;
                            RAVELOG_ERROR(s.str());
                            bfail = true;
                            break;
                        }
                        robot->SubtractActiveDOFValues(*itsol,vrealsolution);
                        dReal diff = 0;
                        FOREACHC(it,*itsol) {
                            diff += *it**it;
                        }
                        if( diff <= fthreshold ) {
                            bfoundinput = true;
                        }
                    }

                    if( bfail ) {
                        ++i;
                        continue;
                    }
                    if( !bfoundinput ) {
                        vnofullsolutions.push_back(make_pair(twrist,vfreeparameters_real));
                    }
                }

                if( pmanip->GetIkSolver()->GetNumFreeParameters() == 0 ) {
                    if( bsuccess ) {
                        success++;
                    }
                    i++;
                    continue;
                }

                // test with the free parameters
                robot->SetActiveDOFValues(vrand, false);
                if( DebugIKFindSolution(pmanip, twrist, viksolution, filteroptions, vfreeparameters, vfreeparameters_out.size()-1) ) {
                    robot->SetActiveDOFValues(viksolution, false);
                    twrist_out = pmanip->GetIkParameterization(twrist);
                    if(twrist.ComputeDistanceSqr(twrist_out) > fthreshold ) {
                        vwrongsolutions.push_back(make_pair(twrist,vfreeparameters));
                        bsuccess = false;
                        s.str("");
                        s << "FindIKSolution (freeparams): Incorrect IK, i = " << i << " error: " << RaveSqrt(twrist.ComputeDistanceSqr(twrist_out)) << endl
                          << "freeparams: ";
                        FOREACH(it, vfreeparameters) {
                            s << *it << " ";
                        }
                        s << endl << "Original Joint Val: ";
                        FOREACH(it, vrealsolution) {
                            s << *it << " ";
                        }
                        s << endl << "Returned Joint Val: ";
                        FOREACH(it, viksolution) {
                            s << *it << " ";
                        }
                        s << endl << "in: " << twrist << endl;
                        s << "out: " << twrist_out << endl << endl;
                        RAVELOG_ERROR(s.str());
                        ++i;
                        continue;
                    }

                    // make sure they are the same
                    if( !pmanip->GetIkSolver()->GetFreeParameters(vfreeparameters_out) ) {
                        RAVELOG_WARN("failed to get freeparameters");
                    }
                    for(int j = 0; j < pmanip->GetIkSolver()->GetNumFreeParameters(); ++j) {
                        if( fabsf(vfreeparameters.at(j)-vfreeparameters_out.at(j)) > 0.0001f ) {
                            RAVELOG_WARN(str(boost::format("free params %d not equal: %f!=%f\n")%j%vfreeparameters[j]%vfreeparameters_out[j]));
                            vnofullsolutions.push_back(make_pair(twrist,vfreeparameters));
                            bsuccess = false;
                            break;
                        }
                    }
                }
                else {
                    // not sure if should record failure here... the free parameters increment are different than the IK one
                }

                // test the multiple solution function
                robot->SetActiveDOFValues(vrand, false);
                viksolutions.resize(0);
                DebugIKFindSolutions(pmanip, twrist, viksolutions, filteroptions, vfreeparameters_out, vfreeparameters_out.size()-1);
                // not sure if should record failure if no solutions found... the free parameters increment are different than the IK one
                bool bfail = false;
                FOREACH(itsol, viksolutions) {
                    robot->SetActiveDOFValues(*itsol, false);
                    twrist_out = pmanip->GetIkParameterization(twrist);
                    if(twrist.ComputeDistanceSqr(twrist_out) > fthreshold ) {
                        vwrongsolutions.push_back(make_pair(twrist,vfreeparameters_out));
                        s.str("");
                        s << "FindIKSolutions (freeparams): Incorrect IK, i = " << i <<" error: " << RaveSqrt(twrist.ComputeDistanceSqr(twrist_out)) << endl
                          << "Original Joint Val: ";
                        FOREACH(it, vrealsolution) {
                            s << *it << " ";
                        }
                        s << endl << "Returned Joint Val: ";
                        FOREACH(it, *itsol) {
                            s << *it << " ";
                        }
                        s << endl << "in: " << twrist << endl;
                        s << "out: " << twrist_out << endl;
                        s << "raw ik command: ";
                        GetIKFastCommand(s, pmanip->GetBase()->GetTransform().inverse()*twrist);
                        FOREACH(itfree,vfreeparameters_out) {
                            s << *itfree << " ";
                        }
                        s << endl << endl;
                        RAVELOG_ERROR(s.str());
                        bfail = true;
                        break;
                    }
                }
                if( bfail ) {
                    ++i;
                    continue;
                }

                if( bsuccess ) {
                    success++;
                }
                i++;
            }
        }

        dReal itotal = dReal(1.0)/nTotalIterations;
        RAVELOG_DEBUG(str(boost::format("DebugIK done, rates %f, %f, %f, %f\n")%(success*itotal)%(vwrongsolutions.size()*itotal)%(vnosolutions.size()*itotal)%(vnofullsolutions.size()*itotal)));
        sout << num_itrs << " " << success << " ";
        FOREACH(itresults,vsolutionresults) {
            sout << itresults->size() << " ";
            FOREACH(it, *itresults) {
                sout << it->first << " ";
                FOREACH(itfree,it->second) {
                    sout << *itfree << " ";
                }
            }
        }

        // restore the ik threshold
        sinputik.str(""); sinputik.clear();
        sinputik << "SetIkThreshold 1e-6";
        pmanip->GetIkSolver()->SendCommand(soutik,sinputik);

        return true;
    }

    static void GetIKFastCommand(std::ostream& o, const IkParameterization& param) {
        switch(param.GetType()) {
        case IKP_Transform6D: {
            TransformMatrix tm = param.GetTransform6D();
            o << tm.m[0] << " " << tm.m[1] << " " << tm.m[2] << " " << tm.trans[0] << " " << tm.m[4] << " " << tm.m[5] << " " << tm.m[6] << " " << tm.trans[1] << " " << tm.m[8] << " " << tm.m[9] << " " << tm.m[10] << " " << tm.trans[2] << " ";
            break;
        }
        case IKP_Rotation3D: {
            TransformMatrix tm = matrixFromQuat(param.GetRotation3D());
            o << tm.m[0] << " " << tm.m[1] << " " << tm.m[2] << " 0 " << tm.m[4] << " " << tm.m[5] << " " << tm.m[6] << " 0 " << tm.m[8] << " " << tm.m[9] << " " << tm.m[10] << " 0 ";
            break;
        }
        case IKP_Translation3D: {
            Vector v = param.GetTranslation3D();
            o << "0 0 0 " << v.x << " 0 0 0 " << v.y << " 0 0 0 " << v.z << " ";
            break;
        }
        case IKP_Direction3D: {
            Vector dir = param.GetDirection3D();
            o << dir.x << " " << dir.y << " " << dir.z << " 0 0 0 0 0 0 0 0 0 ";
            break;
        }
        case IKP_Ray4D: {
            Vector pos = param.GetRay4D().pos;
            Vector dir = param.GetRay4D().dir;
            o << dir.x << " " << dir.y << " " << dir.z << " " << pos.x << " 0 0 0 " << pos.y << " 0 0 0 " << pos.z << " ";
            break;
        }
        case IKP_Lookat3D: {
            Vector v = param.GetLookat3D();
            o << "0 0 0 " << v.x << " 0 0 0 " << v.y << " 0 0 0 " << v.z << " ";
            break;
        }
        case IKP_TranslationDirection5D: {
            Vector dir = param.GetTranslationDirection5D().dir;
            Vector pos = param.GetTranslationDirection5D().pos;
            o << dir.x << " " << dir.y << " " << dir.z << " " << pos.x << " 0 0 0 " << pos.y << " 0 0 0 " << pos.z << " ";
            break;
        }
        case IKP_TranslationXY2D: {
            Vector v = param.GetTranslationXY2D();
            o << "0 0 0 " << v.x << " 0 0 0 " << v.y << " 0 0 0 0 ";
            break;
        }
        case IKP_TranslationXYOrientation3D: {
            Vector v = param.GetTranslationXYOrientation3D();
            o << "0 0 0 " << v.x << " 0 0 0 " << v.y << " 0 0 0 " << v.z << " ";
            break;
        }
        case IKP_TranslationLocalGlobal6D: {
            std::pair<Vector,Vector> p = param.GetTranslationLocalGlobal6D();
            o << p.first.x << " 0 0 " << p.second.x << " 0 " << p.first.y << " 0 " << p.second.y << " 0 0 " << p.first.z << " " << p.second.z << " ";
            break;
        }
        default:
            BOOST_ASSERT(0);
        }
    }

    static list< boost::shared_ptr<IKLibrary> >*& GetLibraries()
    {
        static list< boost::shared_ptr<IKLibrary> >* s_vStaticLibraries=NULL;
        if( s_vStaticLibraries == NULL ) {
            s_vStaticLibraries = new list< boost::shared_ptr<IKLibrary> >();
        }
        return s_vStaticLibraries;
    }

    static boost::mutex& GetLibraryMutex()
    {
        static boost::mutex s_LibraryMutex;
        return s_LibraryMutex;
    }

    /// sinput holds the freeindices and other run-time configuraiton parameters
    static IkSolverBasePtr CreateIkSolver(const string& _name, std::vector<dReal> vfreeinc, EnvironmentBasePtr penv)
    {
        string name; name.resize(_name.size());
        std::transform(_name.begin(), _name.end(), name.begin(), ::tolower);
        /// start from the newer libraries
        boost::mutex::scoped_lock lock(GetLibraryMutex());
        for(list< boost::shared_ptr<IKLibrary> >::reverse_iterator itlib = GetLibraries()->rbegin(); itlib != GetLibraries()->rend(); ++itlib) {
            FOREACHC(itikname,(*itlib)->GetIKNames()) {
                if( name == *itikname ) {
                    return (*itlib)->CreateSolver(penv,vfreeinc);
                }
            }
        }
        return IkSolverBasePtr();
    }
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(IKSolutionFloat)
BOOST_TYPEOF_REGISTER_TYPE(IKSolutionDouble)
BOOST_TYPEOF_REGISTER_TYPE(IKFastProblem)
#endif

#endif
