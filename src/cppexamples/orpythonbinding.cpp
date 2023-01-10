/** \example orpythonbinding.cpp
    \author Rosen Diankov

    Shows how to creating python bindings with an OpenRAVE C++ plugin. The demo registers a python function to be called inside the environment simulation thread using a Module interface.

    The compilation procedure will produce a orpythonbinding shared object or DLL, which can then be directly included into python.

    The following python example will register 'mysimfunction' with the enviornment thread, and run it until it returns true.
   \code{*.py}
   from openravepy import *
   env=Environment()
   RaveSetDebugLevel(DebugLevel.Debug)
   import orpythonbinding
   orpythonbinding.Init(RaveGlobalState())
   totaltime = 0
   def mysimfunction(elapsedtime):
       """return True to end the thread"""
       global totaltime
       totaltime += elapsedtime
       print 'this is the time',totaltime
       return totaltime > 5

   module = orpythonbinding.RegisterSimulationFunction(RaveGetEnvironmentId(env),mysimfunction)
   while True:
       sleep(1)
    \endcode

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

namespace cppexamples {

class FunctionUserData : public UserData
{
public:
    virtual ~FunctionUserData() {
    }
    py::object simulationfn;
};

class PythonBindingModule : public ModuleBase
{
public:
    PythonBindingModule(EnvironmentBasePtr penv, std::istream&) : ModuleBase(penv) {
        SetUserData(UserDataPtr(new FunctionUserData()));
    }
    virtual ~PythonBindingModule() {
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
    return InterfaceBasePtr(new PythonBindingModule(penv,istream));
}

InterfaceBasePtr RegisterSimulationFunction(int environmentid, py::object simulationfn)
{
    ModuleBasePtr module = RaveCreateModule(RaveGetEnvironment(environmentid), "PythonBinding");
    if( !!module ) {
        boost::shared_ptr<FunctionUserData> p = boost::dynamic_pointer_cast<FunctionUserData>(module->GetUserData());
        p->simulationfn = simulationfn;
        module->GetEnv()->Add(module,true,"");
    }
    return InterfaceBasePtr(module);
}

void Init(UserDataPtr globalstate)
{
    RaveInitializeFromState(globalstate);
    if( !g_PythonBindingInterfaceHandle ) {
        g_PythonBindingInterfaceHandle = RaveRegisterInterface(PT_Module, "PythonBinding", NULL, NULL, PythonBindingCreateInterface);
    }
}

} // end namespace cppexamples

BOOST_PYTHON_MODULE(orpythonbinding)
{
    py::def("Init", cppexamples::Init, py::args("globalstate"), "initializes the python bindings with the openrave global state");
    py::def("RegisterSimulationFunction", cppexamples::RegisterSimulationFunction, py::args("environmentid","simulationfn"));
};
