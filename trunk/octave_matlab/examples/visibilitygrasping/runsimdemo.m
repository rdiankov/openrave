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
global probs
addopenravepaths_visibilitygrasping();

randomize = struct('createprob',0.15,'obstacles',{{'data/box0.kinbody.xml','data/box1.kinbody.xml','data/box2.kinbody.xml','data/box3.kinbody.xml'}},'maxcreate',4);
[robot, scenedata] = SetupPA10Scene('data/pa10grasp.env.xml',randomize);
dopause = 0;

while(1)
    orProblemSendCommand('releaseall',probs.manip);
    RobotGoInitial(robot,scenedata.home);

    res = orProblemSendCommand(['MoveToObserveTarget target ' scenedata.targetname ' sampleprob 0.001 sensorindex 0 maxiter 4000 convexfile ' scenedata.convexfile ' visibilitytrans ' scenedata.visibilityfile],probs.visual,1);
    if( isempty(res) )
        warning('failed to move to target');
        continue;
    end

    WaitForRobot(robot.id);

    if(dopause)
        disp('press any key');
        pause;
    end
    %% start visual servoing step
    res = orProblemSendCommand(['VisualFeedbackGrasping target ' scenedata.targetname ' sensorindex 0 convexfile ' scenedata.convexfile ' graspset ' scenedata.graspsetfile '; maxiter 150 visgraspthresh 0.5 gradientsamples 5 '],probs.visual,0);
    if( isempty(res) )
        warning('failed to find visual feedback grasp');
        continue;
    end
    WaitForRobot(robot.id);

    if(dopause)
        disp('press any key');
        pause;
    end

    orProblemSendCommand('closefingers',probs.manip);
    WaitForRobot(robot.id);
    orProblemSendCommand(['grabbody name ' scenedata.targetname],probs.manip);
    
    L = orBodyGetLinks(robot.id);
    Thand = [reshape(L(:,robot.manips{robot.activemanip}.eelink+1),[3 4]); 0 0 0 1];
    Tobj = [reshape(orBodyGetTransform(scenedata.targetid),[3 4]); 0 0 0 1];
    Treltrans = inv(Tobj)*Thand;

    %% arrange distances from farthest to closest
    qorig = QuatFromRotationMatrix(Tobj(1:3,1:3));
    dists = zeros(1,size(scenedata.dests,2));
    for i = 1:size(scenedata.dests,2)
        T = reshape(scenedata.dests(:,i),[3 4]);
        q = QuatFromRotationMatrix(T(1:3,1:3));
        dists(i) = min(sum((q-qorig).^2),sum((q+qorig).^2))*0.2+sum( (Tobj(1:3,4)-T(1:3,4)).^2 )+(rand()*0.1-0.05);
    end

    [s,inds] = sort(-dists);
    dests = scenedata.dests(:,inds);

    %% continue trying to move to a position until success
    success = 0;
    matrices = '';
    for i = 1:size(dests,2)
        T = reshape(dests(:,i),[3 4])*Treltrans;
        matrices = [matrices ' matrix ' sprintf('%f ',T)];
    end
        
    res = orProblemSendCommand(['MoveToHandPosition ' matrices], probs.manip);
    if( ~isempty(res) )
        success = 1;
        WaitForRobot(robot.id);
        break;
    else
        disp('failed, trying again');
        continue;
    end
     
    orRobotSetActiveDOFs(robot.id,robot.manips{robot.activemanip}.handjoints);
    orProblemSendCommand(['releasefingers target ' scenedata.targetname],probs.manip);
    orRobotSetActiveDOFs(robot.id,0:(robot.dof-1));
end
