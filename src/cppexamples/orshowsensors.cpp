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

#include "orexample.h"

using namespace OpenRAVE;
using namespace std;

namespace cppexamples {

class ShowSensorsExample : public OpenRAVEExample
{
public:
    virtual void demothread(int argc, char ** argv) {
        penv->Load("data/testwamcamera.env.xml");
        size_t ienablesensor = 0;
        // get all the sensors, this includes all attached robot sensors
        std::vector<SensorBasePtr> sensors;
        penv->GetSensors(sensors);
        while(IsOk()) {
            for(size_t isensor = 0; isensor < sensors.size(); ++isensor) {
                sensors[isensor]->Configure(isensor == ienablesensor ? SensorBase::CC_PowerOn : SensorBase::CC_PowerOff);
                sensors[isensor]->Configure(isensor == ienablesensor ? SensorBase::CC_RenderDataOn : SensorBase::CC_RenderDataOff);
            }
            ienablesensor = (ienablesensor+1)%sensors.size();
            boost::this_thread::sleep(boost::posix_time::seconds(5));
        }
    }
};

} // end namespace cppexamples

int main(int argc, char ** argv)
{
    cppexamples::ShowSensorsExample example;
    return example.main(argc,argv);
}
