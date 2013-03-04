/** \example orc_bodyfunctions.cpp
    \author Rosen Diankov

    Shows how to change body transforms, remove bodies, and get link information and change geometry properties.
 */
#include <openrave-core_c.h>
#include <cstdio>
#include <malloc.h>
#include <math.h>

int main(int argc, char ** argv)
{
    ORCInitialize();
    void* env = ORCEnvironmentCreate();
    ORCEnvironmentLoad(env, "data/lab1.env.xml");
    ORCEnvironmentSetViewer(env,"qtcoin");

    ORCSetDebugLevel(4); // set to debug

    bool pole2added = true;
    void* bodypole2 = ORCEnvironmentGetKinBody(env, "pole2");
    void* bodypole3 = ORCEnvironmentGetKinBody(env, "pole3");

    int numlinks = ORCBodyGetLinks(bodypole3, NULL);
    void** links = NULL;
    links = (void**)malloc(sizeof(void*)*numlinks);
    ORCBodyGetLinks(bodypole3, links);

    // get the geometries of the first link
    int numgeometries = ORCBodyLinkGetGeometries(links[0], NULL);
    void** geometries = NULL;
    geometries = (void**)malloc(sizeof(void*)*numgeometries);
    ORCBodyLinkGetGeometries(links[0], geometries);

    OpenRAVEReal pose[7];
    ORCBodyGetTransform(bodypole3, pose);
    unsigned long long startsimtime = ORCEnvironmentGetSimulationTime(env);

    for(int i = 0; i < 10000; ++i) {
        // lock the environment when doing operations
        ORCEnvironmentLock(env);

        unsigned long long newsimtime = ORCEnvironmentGetSimulationTime(env);
        OpenRAVEReal deltatime = (newsimtime-startsimtime)*1e-6;
        OpenRAVEReal fanim = fmod(deltatime,1.0);
        // animate around X axis
        pose[4] = 1 + fanim;
        ORCBodySetTransform(bodypole3,pose);

        // set the color
        ORCBodyGeometrySetDiffuseColor(geometries[0], 1,0,fanim);

        if( fanim > 0.5 && pole2added ) {
            // remove the pole
            ORCEnvironmentRemove(env, bodypole2);
            pole2added = false;
        }
        else if( fanim < 0.5 && !pole2added ) {
            ORCEnvironmentAdd(env, bodypole2);
            pole2added = true;
        }

        // unlock
        ORCEnvironmentUnlock(env);

        // wait until sim time changes
        while(newsimtime == ORCEnvironmentGetSimulationTime(env)) {
            // sleep?
        }
    }

    // release all resources
    for(int i = 0; i < numgeometries; ++i) {
        ORCBodyLinkRelease(geometries[i]);
    }
    free(geometries);
    for(int i = 0; i < numlinks; ++i) {
        ORCBodyLinkRelease(links[i]);
    }
    free(links);
    ORCInterfaceRelease(bodypole2);
    ORCInterfaceRelease(bodypole3);
    ORCEnvironmentDestroy(env);
    ORCEnvironmentRelease(env);
    ORCDestroy();
};
