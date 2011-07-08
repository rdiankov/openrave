/** \example orshowsensors.cpp
    \author Rosen Diankov

    Shows how to toggle sensor power and rendering options

    \image html showsensors_camera.jpg "Camera Sensor."
    \image latex showsensors_camera.jpg "Camera Sensor." width=20cm

    \image html showsensors_laser.jpg "Laser Sensor."
    \image latex showsensors_laser.jpg "Laser Sensor." width=20cm

    <b>Full Example Code:</b>
 */
#include <openrave-core.h>
#include <vector>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#define usleep(micro) Sleep(micro/1000)
#endif

using namespace OpenRAVE;
using namespace std;

void SetViewer(EnvironmentBasePtr penv, const string& viewername)
{
    ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
    BOOST_ASSERT(!!viewer);

    // attach it to the environment:
    penv->AddViewer(viewer);

    // finally you call the viewer's infinite loop (this is why you need a separate thread):
    bool showgui = true;
    viewer->main(showgui);
}

int main(int argc, char ** argv)
{
    RaveInitialize(true); // start openrave core
    EnvironmentBasePtr penv = RaveCreateEnvironment(); // create the main environment
    boost::thread thviewer(boost::bind(SetViewer,penv,"qtcoin"));
    penv->Load("data/testwamcamera.env.xml");
    size_t ienablesensor = 0;
    // get all the sensors, this includes all attached robot sensors
    std::vector<SensorBasePtr> sensors;
    penv->GetSensors(sensors);
    while(1) {
        for(size_t isensor = 0; isensor < sensors.size(); ++isensor) {
            sensors[isensor]->Configure(isensor == ienablesensor ? SensorBase::CC_PowerOn : SensorBase::CC_PowerOff);
            sensors[isensor]->Configure(isensor == ienablesensor ? SensorBase::CC_RenderDataOn : SensorBase::CC_RenderDataOff);
        }
        ienablesensor = (ienablesensor+1)%sensors.size();
        usleep(5000000); // 5s
    }
    return 0;
}
