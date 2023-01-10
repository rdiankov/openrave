/** \example orconveyormovement.cpp
    \author Rosen Diankov

    \image html cppexample_orconveyormovement.jpg "Parts moving on a conveyor belt."
    \image latex cppexample_orconveyormovement.jpg "Parts moving on a conveyor belt." width=10cm

    Shows how to setup a simulation loop to move objects around a conveyor belt.

    <b>Full Example Code:</b>
 */
#include <openrave-core.h>
#include <vector>
#include <sstream>

#include "orexample.h"

using namespace OpenRAVE;
using namespace std;

namespace cppexamples {

/// Holds a registered set of bodies, at every time step creates new bodies and moves them along a trajectory.
class ConveyorBeltModule : public ModuleBase
{
    struct RegisteredBody
    {
        string filename;
        dReal appearanceprobability;     // probably of appearance in 1 second
    };

    struct InstancedBody
    {
        KinBodyPtr pbody;
        dReal timeleft;
    };
    SpaceSamplerBasePtr _psampler;
public:
    ConveyorBeltModule(EnvironmentBasePtr penv, std::istream& is) : ModuleBase(penv)
    {
        __description = "Handles conveyor belt movement";
        RegisterCommand("registerbody",boost::bind(&ConveyorBeltModule::RegisterBody,this,_1,_2),"registers a body to be put into the environment");
        movevel = Vector(0,0.4,0);
        start = Vector(0.5,-1,0.6);
        _psampler = RaveCreateSpaceSampler(penv,"mt19937");
    }

    int main(const string& cmd)
    {
        return 0;
    }

    bool RegisterBody(ostream& sout, istream& sinput)
    {
        EnvironmentLock lock(GetEnv()->GetMutex());
        RegisteredBody body;
        sinput >> body.filename >> body.appearanceprobability;
        if( !sinput ) {
            return false;
        }
        _listregistered.push_back(body);
        return true;
    }

    bool SimulationStep(dReal fElapsedTime)
    {
        for(list<RegisteredBody>::iterator it = _listregistered.begin(); it != _listregistered.end(); ++it) {
            // appearanceprobabiliy is in seconds, so have to transform
            dReal appearanceprobability = 1-pow(1-it->appearanceprobability,fElapsedTime);
            vector<dReal> vsample;
            _psampler->SampleSequence(vsample,4,IT_OpenStart);
            if( vsample.at(0) < appearanceprobability ) {
                KinBodyPtr pbody = GetEnv()->ReadKinBodyXMLFile(it->filename);
                GetEnv()->Add(pbody,true);
                InstancedBody b;
                for(int iter = 0; iter < 10; ++iter) {
                    Transform t;
                    t.rot = geometry::quatFromAxisAngle<dReal>(Vector(0,0,1),vsample.at(1)*2*PI);
                    t.trans = start + Vector(vsample.at(2)-0.5,vsample.at(3)-0.5,0)*0.4;
                    pbody->SetTransform(t);
                    if( !GetEnv()->CheckCollision(KinBodyConstPtr(pbody)) ) {
                        b.pbody = pbody;
                        break;
                    }
                }

                if( !b.pbody ) {
                    GetEnv()->Remove(pbody);
                }
                else {
                    b.timeleft = 4.0;
                    _listinstances.push_back(b);
                }
            }
        }

        list<InstancedBody>::iterator it = _listinstances.begin();
        while(it != _listinstances.end() ) {
            Transform t = it->pbody->GetTransform();
            t.trans += fElapsedTime*movevel;
            it->pbody->SetTransform(t);
            it->timeleft -= fElapsedTime;
            if( it->timeleft <= 0 ) {
                GetEnv()->Remove(it->pbody);
                it = _listinstances.erase(it);
            }
            else {
                ++it;
            }
        }
        return false;
    }

    static InterfaceBasePtr create(EnvironmentBasePtr penv, std::istream& is)
    {
        return InterfaceBasePtr(new ConveyorBeltModule(penv,is));
    }

private:
    Vector start, movevel;
    list<RegisteredBody> _listregistered;
    list<InstancedBody> _listinstances;
};

class ConveyorExample : public OpenRAVEExample
{
public:
    virtual void demothread(int argc, char ** argv) {
        boost::shared_ptr<void> handle = RaveRegisterInterface(PT_Module,"conveyorbelt", NULL, NULL, ConveyorBeltModule::create);

        // load the environment
        string scenefilename = "robots/pr2-beta-static.zae";
        penv->Load(scenefilename);

        // create the conveyor module and add a couple of bodies for simulation
        ModuleBasePtr p = RaveCreateModule(penv,"conveyorbelt");
        penv->Add(p,true,"");
        stringstream sout, sin("registerbody data/mug1.kinbody.xml 0.6");
        p->SendCommand(sout,sin);
        sin.clear();
        sin.str("registerbody data/ketchup.kinbody.xml 0.3");
        p->SendCommand(sout,sin);
        while(IsOk()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
};

} // end namespace cppexamples

int main(int argc, char ** argv)
{
    cppexamples::ConveyorExample example;
    return example.main(argc,argv);
}
