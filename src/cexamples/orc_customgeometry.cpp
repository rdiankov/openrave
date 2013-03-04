/** \example orc_customgeometry.cpp
    \author Rosen Diankov

    Shows how to create custom geometry dynamically and rotate it. This geometry is now part of the collision detection.
 */
#include <openrave-core_c.h>
#include <cstdio>
#include <malloc.h>
#include <math.h>

int main(int argc, char ** argv)
{
    ORCInitialize();
    void* env = ORCEnvironmentCreate();
    //ORCEnvironmentLoad(env, "data/lab1.env.xml");
    ORCSetDebugLevel(4); // set to debug

    // create a simple triangle
    OpenRAVEReal vertices[9]={1,0,0, 0,1,0,  0,0,1 };
    int numvertices=3;
    OpenRAVEReal indices[3]={0,1,2};
    int numtriangles=1;
    void* trimesh = ORCCreateTriMesh(vertices, numvertices, indices, numtriangles);

    // create a body to encompass the triangle
    void* body = ORCCreateKinBody(env, "");
    ORCBodyInitFromTrimesh(body, trimesh,true);
    ORCBodySetName(body, "mytriangle");
    ORCEnvironmentAdd(env,body);

    // attach the viewer
    ORCEnvironmentSetViewer(env,"qtcoin");

    // animate along Z axis
    OpenRAVEReal pose[7] = {1,0,0,0,0,0,0};
    unsigned long long startsimtime = ORCEnvironmentGetSimulationTime(env);

    for(int i = 0; i < 10000; ++i) {
        // lock the environment when doing operations
        ORCEnvironmentLock(env);

        unsigned long long newsimtime = ORCEnvironmentGetSimulationTime(env);
        OpenRAVEReal deltatime = (newsimtime-startsimtime)*1e-6;
        OpenRAVEReal fanim = fmod(deltatime,1.0);

        pose[0] = sin(deltatime);
        pose[3] = cos(deltatime);
        ORCBodySetTransform(body,pose);

        // unlock
        ORCEnvironmentUnlock(env);

        // wait until sim time changes
        while(newsimtime == ORCEnvironmentGetSimulationTime(env)) {
            // sleep?
        }
    }

    ORCTriMeshDestroy(trimesh);
    ORCInterfaceRelease(body);
    ORCEnvironmentDestroy(env);
    ORCEnvironmentRelease(env);
    ORCDestroy();
};
