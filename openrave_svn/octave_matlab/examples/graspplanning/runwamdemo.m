% Copyright (C) 2008-2010 Rosen Diankov
% 
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
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
    curobj = scenedata.targetobjs{floor(length(scenedata.targetobjs)*rand*0.9999+1)};
    curobj.grasps = scenedata.GraspTable;
    curobj.dests = scenedata.dests;

    orProblemSendCommand('releaseall',probs.manip);
    [graspsuccess, full_solution_index] = GraspAndPlaceObject(robot, curobj, squeeze, MySwitchModels,SwitchModelPatterns);
    WaitForRobot(robot.id);
    if( ~graspsuccess )
        warning('failed grasp');
        continue;
    end
end
