/** \file orexample.h
    \author Rosen Diankov

    Included by most OpenRAVE C++ examples.
 */
#include <openrave-core.h>
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <signal.h>

namespace cppexamples {

/** \brief A simple framework for running C++ examples.

    It makes sure OpenRAVE cleans itself up by registering itself into the sighandler. In order to use, derive from it:

    \code
    class TrajectoryExample : public OpenRAVEExample
    {
    public:
        virtual void demothread() {
            // insert user code here
            penv->Load("data/lab1.env.xml");
        }
    };
    \endcode
 */
class OpenRAVEExample
{
public:
    OpenRAVEExample(const std::string& viewername="qtcoin") : _viewername(viewername), _bDestroyThread(false) {
        OPENRAVE_ASSERT_FORMAT0(GetSingleton()==NULL,"expecting only once instance of OpenRAVEExample",OpenRAVE::ORE_Assert);
        GetSingleton() = this;
        signal(SIGINT,sigint_handler); // for control C
    }
    virtual ~OpenRAVEExample()
    {
        GetSingleton() = NULL;
        this->Exit();
        if( _thopenrave.joinable() ) { //!!_thopenrave ) {
            _thopenrave.join(); // wait for the thread to exit
        }
    }

    virtual int main(int argc, char ** argv)
    {
        OpenRAVE::RaveInitialize(true);
        penv=OpenRAVE::RaveCreateEnvironment();
        penv->SetDebugLevel(OpenRAVE::Level_Debug);

        // create a viewer
        _viewer.reset();
        if( _viewername.size() > 0 ) {
            _viewer = OpenRAVE::RaveCreateViewer(penv,_viewername);
        }
        if( !!_viewer ) {
            penv->Add(_viewer);

            // create the main openrave thread
            _bDestroyThread = false;
            _thopenrave = boost::thread(boost::bind(&OpenRAVEExample::_demothreadwrapper,this,argc,argv));

            // start the viewer main loop
            _viewer->main(true);
        }
        else {
            // just execute
            demothread(argc,argv);
        }
        return 0;
    }

    virtual bool IsOk() {
        return !_bDestroyThread;
    }

    virtual void Exit() {
        _bDestroyThread = true;
        OpenRAVE::RaveDestroy();
    }

    virtual void demothread(int argc, char ** argv) = 0;

protected:
    OpenRAVE::EnvironmentBasePtr penv;

private:
    OpenRAVE::ViewerBasePtr _viewer;
    std::string _viewername;
    bool _bDestroyThread;
    boost::thread _thopenrave;

    void quitviewer(void *) {
        if( !!_viewer ) {
            _viewer->quitmainloop();
        }
    }

    void _demothreadwrapper(int argc, char ** argv) {
        boost::shared_ptr<void> quitviewer((void*)NULL, boost::bind(&OpenRAVEExample::quitviewer, this,_1));
        demothread(argc,argv);
    }

    static void sigint_handler(int sig)
    {
        if( !!OpenRAVEExample::GetSingleton() ) {
            OpenRAVEExample::GetSingleton()->Exit();
        }
    }

    static OpenRAVEExample*& GetSingleton() {
        static OpenRAVEExample* psingleton = NULL;
        return psingleton;
    }
};

} // end namespace cppexamples
