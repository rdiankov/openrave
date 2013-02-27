/** \example orcrrtplanning.cpp
    \author Rosen Diankov

    Shows how to start a planning with the OpenRAVE C bindings.
 */

#include <openrave-core_c.h>
#include <cstdio>
#include <malloc.h>

int main(int argc, char ** argv)
{
    void* env = ORCEnvironmentCreate();
    ORCEnvironmentLoad(env, "data/lab1.env.xml");

    ORCSetDebugLevel(4); // set to debug

    int numbodies = ORCEnvironmentGetBodies(env, NULL);
    void** bodies = NULL;
    bodies = (void**)malloc(sizeof(void*)*numbodies);
    ORCEnvironmentGetBodies(env, bodies);

    for(int i = 0; i < numbodies; ++i) {
        const char* bodyname = ORCBodyGetName(bodies[0]);
        printf("%s\n", bodyname);
    }

    // lock the environment when doing operations
    ORCEnvironmentLock(env);

    int numlinks = ORCBodyGetLinks(env, NULL);
    void** links = NULL;
    links = (void**)malloc(sizeof(void*)*numlinks);
    ORCBodyGetLinks(env, links);

    // unlock
    ORCEnvironmentUnlock(env);

    free(bodies);
    ORCEnvironmentRelease(env);
};
