/** \example orc_rrtplanning.cpp
    \author Rosen Diankov

    Shows how to start a planning with the OpenRAVE C bindings.
 */
#include <openrave-core_c.h>
#include <cstdio>
#include <malloc.h>

int main(int argc, char ** argv)
{
    ORCInitialize();
    void* env = ORCEnvironmentCreate();
    ORCEnvironmentLoad(env, "data/lab1.env.xml");

    ORCSetDebugLevel(4); // set to debug

    int numrobots = ORCEnvironmentGetRobots(env, NULL);
    void** robots = NULL;
    robots = (void**)malloc(sizeof(void*)*numrobots);
    ORCEnvironmentGetRobots(env, robots);

    const char* robotname = ORCRobotGetName(robots[0]);
    printf("robot name is: %s\n", robotname);

    void* basemanip = ORCModuleCreate(env, "BaseManipulation");
    ORCEnvironmentAddModule(env, basemanip, robotname);
    char* output = ORCInterfaceSendCommand(basemanip, "MoveManipulator goal -0.75 1.24 -0.064 2.33 -1.16 -1.548 1.19 outputtraj execute 0");
    printf("rrt output is: %s", output);

    // release all resources
    free(output);
    ORCInterfaceRelease(basemanip);
    for(int i = 0; i < numrobots; ++i) {
        ORCInterfaceRelease(robots[i]);
    }
    free(robots);
    ORCEnvironmentDestroy(env);
    ORCEnvironmentRelease(env);
    ORCDestroy();
};
