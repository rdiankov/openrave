/** \example orpythonbinding.cpp
    \author Rosen Diankov

    Shows how to creating python bindings with an OpenRAVE C++ plugin. The demo registers a python function to be called inside the environment simulation thread using a Problem Instance.

    The compilation procedure will produce a orpythonbinding shared object or DLL, which can then be directly included into python.
    
    The following python example will register 'mysimfunction' with the enviornment thread, and run it until it returns true.
    \verbatim
from openravepy import *
env=openravepy.Environment()
RaveSetDebugLevel(DebugLevel.Debug)
import orpythonbinding
orpythonbinding.Init(RaveGlobalState())
totaltime = 0
def mysimfunction(elapsedtime):
    global totaltime
    totaltime += elapsedtime
    print 'this is the time',totaltime
    # return True to end the thread
    return totaltime > 5

prob = orpythonbinding.RegisterSimulationFunction(RaveGetEnvironmentId(env),mysimfunction)
while True:
    sleep(1)
    \endverbatim

    <b>Full Example Code:</b>
*/
#include <openrave/openrave.h>

#include <boost/python.hpp>
#include <boost/python/exception_translator.hpp>
#include <boost/python/stl_iterator.hpp>
#include <pyconfig.h>

#include <exception>
#include <boost/shared_ptr.hpp>
#include <boost/format.hpp>
#include <boost/assert.hpp>

#include <vector>
#include <cstring>
#include <sstream>

using namespace OpenRAVE;
using namespace std;

class FunctionUserData : public UserData
{
public:
    virtual ~FunctionUserData() {}
    boost::python::object simulationfn;
};

class PythonBindingProblemInstance : public ProblemInstance
{
public:
    PythonBindingProblemInstance(EnvironmentBasePtr penv, std::istream&) : ProblemInstance(penv) {
        SetUserData(UserDataPtr(new FunctionUserData()));
    }
    virtual ~PythonBindingProblemInstance() {
        RAVELOG_DEBUG("destroying python binding\n");
    }

    virtual bool SimulationStep(dReal fElapsedTime) {
        boost::shared_ptr<FunctionUserData> p = boost::dynamic_pointer_cast<FunctionUserData>(GetUserData());
        bool ret = false;
        if( !!p ) {
            PyGILState_STATE gstate = PyGILState_Ensure();
            try {
                ret = p->simulationfn(fElapsedTime);
            }
            catch(...) {
                RAVELOG_WARN("unknown exception in python callback, please register again:\n");
                PyErr_Print();
                ret = true;
            }
            PyGILState_Release(gstate);
            if( ret ) {
                GetEnv()->Remove(shared_from_this());
            }
        }
        return ret;
    }
};

boost::shared_ptr<void> g_PythonBindingInterfaceHandle;

InterfaceBasePtr PythonBindingCreateInterface(EnvironmentBasePtr penv, std::istream& istream)
{
    return InterfaceBasePtr(new PythonBindingProblemInstance(penv,istream));
}

InterfaceBasePtr RegisterSimulationFunction(int environmentid, boost::python::object simulationfn)
{
    ProblemInstancePtr prob = RaveCreateProblem(RaveGetEnvironment(environmentid), "PythonBinding");
    if( !!prob ) {
        boost::shared_ptr<FunctionUserData> p = boost::dynamic_pointer_cast<FunctionUserData>(prob->GetUserData());
        p->simulationfn = simulationfn;
        prob->GetEnv()->LoadProblem(prob,"");
    }
    return InterfaceBasePtr(prob);
}

void Init(UserDataPtr globalstate)
{
    RaveInitializeFromState(globalstate);
    if( !g_PythonBindingInterfaceHandle ) {
        g_PythonBindingInterfaceHandle = RaveRegisterInterface(PT_ProblemInstance, "PythonBinding", OPENRAVE_PROBLEM_HASH,OPENRAVE_ENVIRONMENT_HASH, PythonBindingCreateInterface);
    }
}

BOOST_PYTHON_MODULE(orpythonbinding)
{
    boost::python::def("Init", Init, boost::python::args("globalstate"), "initializes the python bindings with the openrave global state");
    boost::python::def("RegisterSimulationFunction", RegisterSimulationFunction, boost::python::args("environmentid","simulationfn"));
};
