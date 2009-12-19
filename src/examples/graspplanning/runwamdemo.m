more off;
addopenravepaths_graspplanning();
global probs
orEnvSetOptions('debug 4');

scenefile = 'data/lab1.env.xml';
grasptablefile = '../grasping/grasptables/grasp_barrett_mug1.mat';
[robot, scenedata] = SetupSceneFromGraspTables(scenefile,grasptablefile);

armjoints = robot.manips{robot.activemanip}.armjoints;
handjoints = robot.manips{robot.activemanip}.handjoints;
wristlinkid = robot.manips{robot.activemanip}.eelink;

orProblemSendCommand('releaseall',probs.manip);
RobotGoInitial(robot,scenedata.home);

squeeze = [];
MySwitchModels = @(x) 1;
SwitchModelPatterns = {};

while(1)
    curobj = scenedata.targetobjs{1};%{floor(length(scenedata.targetobjs)*rand*0.9999+1)};
    curobj.grasps = scenedata.GraspTable;
    curobj.dests = scenedata.dests;

    orProblemSendCommand('releaseall',probs.manip);
    [graspsuccess, full_solution_index] = GraspAndPlaceObject(robot, curobj, squeeze, MySwitchModels,SwitchModelPatterns);
    WaitForRobot(robot.id);
    if( ~graspsuccess )
        warning('failed grasp');
        continue;
    end

    orRobotSetActiveDOFs(robot.id,handjoints);
    orProblemSendCommand(['ReleaseFingers target ' curobj.name],probs.manip);
    orProblemSendCommand('releaseall',probs.manip);
    
    orProblemSendCommand(['movemanipulator armvals ' sprintf('%f ',initvals)], probs.manip);
    WaitForRobot(robot.id);
end
