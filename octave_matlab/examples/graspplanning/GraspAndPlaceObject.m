% [success, full_solution_index] = GraspAndPlaceObject(robot, curobj, squeeze, MySwitchModels, SwitchModelPatterns)
%
% finds a grasp for the current object depending on the environment
% The object will be placed on the table
% curobj has the following members
% - id - unique identifier
% - name - openrave name of obj
% - fatfilename - the filename of the padded object
% - grasps - the grasp table
% - axis - axis used to compute the distance map
% - dests - 12xn matrix of possible destinations of the object. Each column
%           is a 3x4 matrix, get by reshape(dests(:,i),[3 4])

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
function [graspsuccess, full_solution_index] = GraspAndPlaceObject(robot, curobj, squeeze, MySwitchModels, SwitchModelPatterns)

global updir probs

if( isempty(updir) )
    updir = [0;0;1];
end

test = 0;
graspsuccess = 0;

full_solution_index = -1;

disp(sprintf('picking up object %s', curobj.name));

%switch back to real models
if( MySwitchModels(0) )
    curobj.id = orEnvGetBody(curobj.name);
end

% get the transformation
TargTrans = [reshape(orBodyGetTransform(curobj.id), [3 4]); 0 0 0 1];

grasps = curobj.grasps;
order = 1:size(grasps,2);
curgrasp = 1;

armjoints = robot.manip.armjoints;
handjoints = robot.manip.handjoints;
wristlinkid = robot.manip.eelink+1;
robotid = robot.id;

while(curgrasp < size(grasps,1))
    g = grasps(:,curgrasp:end);

    approachoffset = 0.02;
    basetime = toc;
    cmd = ['GraspPlanning maxiter 1000 execute 0 outputtraj ' ...
           ' seedik 1 seedgrasps 3 seeddests 8 randomdests 1 randomgrasps 1 ' ...
           ' target ' curobj.name ...
           ' approachoffset ' sprintf('%f',approachoffset) ...
           ' igraspdir ' sprintf('%d ',robot.grasp.direction(1)) ...
           ' igrasppos ' sprintf('%d ',robot.grasp.center(1)) ...
           ' igrasproll ' sprintf('%d ',robot.grasp.roll(1)) ...
           ' igraspstandoff ' sprintf('%d ',robot.grasp.standoff(1)) ...
           ' igrasppreshape ' sprintf('%d ',robot.grasp.joints(1)) ...
           ' matdests ' num2str(size(curobj.dests,2)) ' ' sprintf('%f ', curobj.dests) ...
           ];
                                     
    for i = 1:length(SwitchModelPatterns)
        cmd = [cmd ' switch ' SwitchModelPatterns{i}.pattern ' ' SwitchModelPatterns{i}.fatfilename ' '];
    end

    cmd = [cmd ' grasps ' num2str(size(g,2)) ' ' num2str(size(g,1)) ' '];
    response = orProblemSendCommand([cmd sprintf('%f ', g)], probs.task,1);
    if( isempty(response) )
	    disp(['failed to find grasp for object ' curobj.name]);
        return;
    end

    %% parse the response
    [numgoals, rem] = ReadValsFromString(response,1);
    [destgoals, rem] = ReadValsFromString(rem,8*numgoals);
    destgoals = reshape(destgoals,[8 numgoals]);
    destgoals = destgoals(2:8,:)

    [graspindex,rem] = ReadValsFromString(rem,1);
    [searchtime,trajdata] = ReadValsFromString(rem,1);
    curgrasp = curgrasp + graspindex;

	putsuccess=1;
    grasp = grasps(:,curgrasp);
    disp(['grasp: ' sprintf('%d ', [curgrasp order(curgrasp)]) ' time: ' sprintf('%fs', toc-basetime)]);
    
    curgrasp = curgrasp+1; % want the next grasp

    if(test)
        return;
    end

    % start the trajectory
    success = StartTrajectory(robotid,trajdata);
    if( ~success )
        warning('failed to start initial traj');
        return;
    end

    disp('moving hand');
    L = orBodyGetLinks(robot.id);
    Tee = reshape(L(:,wristlinkid),[3 4]) * [robot.manip.Tgrasp; 0 0 0 1];
    globalpalmdir = Tee(1:3,1:3)*robot.manip.palmdir;
    movesteps = floor(approachoffset/0.001);
    trajdata = orProblemSendCommand(['MoveHandStraight ignorefirstcollision 0 execute 0 outputtraj direction ' sprintf('%f ',globalpalmdir) ' stepsize 0.001 minsteps ' num2str(movesteps-2) ' maxsteps ' num2str(movesteps+1)], probs.manip);
    if(isempty(trajdata) )
        disp('failed to move greedily, planning instead...')
        Tfinalgrasp = TargTrans*[reshape(grasp(end-11:end),[3 4]); 0 0 0 1];
        trajdata = orProblemSendCommand(['MoveToHandPosition maxiter 1000 maxtries 1 seedik 4 execute 0 outputtraj matrices 1 ' sprintf('%f ', Tfinalgrasp(1:3,1:4))], probs.manip);
    end
    if( isempty(trajdata) )
        error('failed to movehandstraight');
    end
    success = StartTrajectory(robotid,trajdata);
    if( ~success )
        warning('failed to move hand straight');
        return;
    end

    if( ~isempty(squeeze) )
        if( ~squeeze(robot,curobj.name) )
            error('failed to start torque');
        end
    else
        disp('closing fingers');
        %closeoffset = 0.12;
        trajdata = orProblemSendCommand(['CloseFingers execute 0 outputtraj'] , probs.task);
        if( isempty(trajdata) )
            warning('failed to close fingers');
        end

%         %% hack for now
%         [trajvals,transvals,timevals] = ParseTrajData(trajdata);
%         trajvals(41,end) = 0.35;
%         newtrajdata = [sprintf('%d %d 13 ',size(trajvals,2), size(trajvals,1)) sprintf('%f ',[timevals;trajvals;transvals])];
        success = StartTrajectory(robotid,trajdata);
        if( ~success )
            warning('failed to close fingers');
            return;
        end
    end

%     if( MySwitchModels(1) )
%          curobj.id = orEnvGetBody(curobj.name);
%     end

    orProblemSendCommand(['grabbody name ' curobj.name], probs.manip);
    trajdata = orProblemSendCommand(['MoveHandStraight execute 0 outputtraj stepsize 0.003 minsteps 1 ' ...
                                               ' maxsteps 60 direction ' sprintf('%f ', updir)], probs.manip);
    if( isempty(trajdata) )
        warning('failed to movehandstraight');
    end

    %% not going to work well
    %ExecuteOnRealSession(@() orProblemSendCommand(['grabbody name ' curobj.name], probs.manip));

    success = StartTrajectory(robotid,trajdata);
    if( ~success )
        warning('failed to move hand straight');
        return;
    end
    
    squeezesuccess = 1;
% 
%     if( squeezesuccess && squeeze )
%         disp('checking for squeeze failures');
%         curvalues = orBodyGetJointValues(robotid,handjoints);
%         if( all(abs(curvalues)<= 0) )
%             disp(['squeeze failed (' sprintf('%f ', curvalues) ', releasing cup']);
%             squeezesuccess = 0;
%         else
%             disp('squeeze successful after liftoff');
%         end
%     end

    if( size(destgoals,2) == 0 )
        graspsuccess = 1;
        return;
    end

    if( squeezesuccess > 0 )
        disp('planning to destination');
        trajdata = orProblemSendCommand(['MoveToHandPosition maxiter 1000 maxtries 1 seedik 4 execute 0 outputtraj poses ' sprintf('%d ', size(destgoals,2)) sprintf('%f ', destgoals)], probs.manip);
        if( ~isempty(trajdata) )
            success = StartTrajectory(robotid,trajdata);
            if( ~success )
                warning('failed to start trajectory');
                trajdata = orProblemSendCommand(['ReleaseFingers execute 0 outputtraj target ' curobj.name], probs.task);
                success = StartTrajectory(robotid,trajdata);
                return;
            end
        else
            putsuccess = 0;
            warning('failed to move hand');
            return;
        end

        % after trajectory is done, check for failure of grasp (could have
        % dropped cup)
%         if( putsuccess & squeeze )
%             disp('checking for squeeze failures');
% 
%             startoffset = robot.totaldof-robot.handdof;
%             sizeconfig = length(robot.closed_config);
%             curvalues = orRobotGetDOFValues(robotid,startoffset:(startoffset+sizeconfig-1));
%             if( all(curvalues > robot.closed_config') )
%                 % completely failed 
%                 disp(['squeeze failed (' sprintf('%f ', curvalues') ', releasing cup']);
%                 orProblemSendCommand('releaseall');
%                 orRobotControllerSend(robotid, 'ignoreproxy');
%                 % first open the fingers
%                 orProblemSendCommand(['ReleaseFingers target ' curobj.name ...
%                     ' open ' num2str(length(handrobot.open_config)) sprintf('%f ', handrobot.open_config) ...
%                     num2str(handjoints)]);
%                 WaitForRobot(robotid);
%                 RobotGoInitial(robot);
%                 WaitForRobot(robotid);
%                 % enable the object again (the vision system will delete it)
%                 orProblemSendCommand(['visionenable ' num2str(curobj.id) ' 1']);
%                 pause(0.5);
%                 return;
%             else
%                 disp('squeeze successful after liftoff');
%             end
%         end
    end % squeeze success
    
    % even if putsuccess==0 or squeezesuccess==0, need to move the object down and release it,
    % and lift the arm back up
        
    % move the hand down until collision
    disp('moving hand down');
    trajdata = orProblemSendCommand(['MoveHandStraight ignorefirstcollision 0 execute 0 ' ...
                                     ' outputtraj direction ' sprintf('%f ', -updir) ...
                                     ' maxsteps 100 '],probs.manip);
    if( isempty(trajdata) )
        warning('failed to movehandstraight');
    else
        success = StartTrajectory(robotid,trajdata);
        if( ~success )
            warning('failed to move hand straight');
            return;
        end
    end

    disp('opening hand');

    % reenable hand joints
    %orRobotControllerSend(robotid, 'ignoreproxy');
    trajdata = orProblemSendCommand(['ReleaseFingers execute 0 outputtraj target ' curobj.name], probs.task);

    %% cannot wait forever since hand might get stuck
    if( isempty(trajdata) )
        warning('problems releasing, releasing target first');
        orProblemSendCommand('releaseall', probs.manip);
        trajdata = orProblemSendCommand(['ReleaseFingers execute 0 outputtraj target ' curobj.name], probs.task);
        if( isempty(trajdata) )
            warning('failed to release fingers, forcing open');
            orBodySetJointValues(robotid,grasp(robot.grasp.joints),handjoints);
        else
            success = StartTrajectory(robotid,trajdata,4);
        end
    else
        success = StartTrajectory(robotid,trajdata,4);
    end
    
    if( ~success )
        warning('trajectory failed to release fingers');
        return;
    end

    %% now release object
    orProblemSendCommand('releaseall', probs.manip);

    if( orEnvCheckCollision(robotid) )
        %% move back a little (yes, this disregards the target if already in collision)
        trajdata = orProblemSendCommand(['MoveHandStraight execute 0 outputtraj direction ' sprintf('%f ',-globalpalmdir) ' stepsize 0.001 minsteps 1 maxsteps 10'], probs.manip);
        success = StartTrajectory(robotid,trajdata);
    
        if( orEnvCheckCollision(robotid) )
            orProblemSendCommand(['grabbody name ' curobj.name], probs.manip);
            trajdata = orProblemSendCommand(['ReleaseFingers execute 0 outputtraj target ' curobj.name], probs.task);
            success = StartTrajectory(robotid,trajdata,2);
        end
    end

    if( squeezesuccess > 0 && putsuccess > 0 )
        disp('success, putting down');

        % only break when succeeded        
        %orProblemSendCommand(['MoveHandStraight stepsize 0.003 minsteps ' sprintf('%f ', 90) ' maxsteps ' sprintf('%f ', 100) ' direction ' sprintf('%f ', updir')]);
        %RobotGoInitial(robot);
        
        if( MySwitchModels(0) )
            curobj.id = orEnvGetBody(curobj.name);
        end

        graspsuccess = 1;
        break;
    else
        disp('failed');
        
        % go to initial
        RobotGoInitial(robot);
        
        if( MySwitchModels(0) )
            curobj.id = orEnvGetBody(curobj.name);
        end
    end
end

if(test)
    disp('failed to find successful grasp');
    return;
end
