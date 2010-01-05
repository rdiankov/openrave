% [GraspTable, GraspStats] = MakeGraspTable(robot,Target,preshapes,standoffs,rolls,use_noise,stepsize,add_spherenorms)
% 
% this script executes the grasper planner for one environment
% filename is the environment file that should be loaded
% preshapes (optional) - Dxn table specifying n preshape joint configurations. Default is all 0s
% standoffs (optional) - 1xN array of the standoffs to test. Default is [0 0.025]
% rolls (optional) - 1xK array of roll values to test. Default is [0:pi/4:7/4*pi;]
% GraspTable - is a NxD vector containing N grasps parameters
% GraspStats - NxM vector containing various statistics for each grasp

% Copyright (C) 2008-2010 Rosen Diankov, Dmitry Berenson
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
function [GraspTable, GraspStats] = MakeGraspTable(robot,Target,preshapes,standoffs,rolls,use_noise,stepsize,add_spherenorms)

if( ~exist('preshapes','var') )
    preshapes = zeros(length(robot.grasp.joints),1);
end
if( ~exist('standoffs','var') )
    standoffs = [0 0.025];
end
if( ~exist('rolls','var') )
    rolls = 0:pi/2:7/4*pi;
end
if( ~exist('use_noise','var') )
    use_noise = 0;
end
if( ~exist('stepsize','var') )
    stepsize = 0.02; % make step size smaller to increase samples on target surface 
end
if( ~exist('add_spherenorms','var') )
    add_spherenorms = 0;
end
if( size(preshapes,1) ~= length(robot.grasp.joints) )
    error('preshape sizes not equal to robot.grasp.joints');
end

friction_mu = 0.4; % make bigger to increase friction

GraspTable = [];
GraspStats = [];

if(~use_noise)
    num_tries = 1;
else
    num_tries = 5;
end

orEnvSetOptions('collision ode');

orBodySetTransform(Target.id, [0 0 0],[1 0 0 0]);
ab = orBodyGetAABB(Target.id);

% get approach directions
ApproachDirs = GetBoxApproachDirections(ab(:,1), 2*ab(:,2), stepsize);

% orEnvPlot(transpose(ApproachDirs(1:3,:)), 'size',5,'color',[1 0 0]);
% edges = [1 1 1; -1 1 1; -1 -1 1; 1 -1 1; 1 1 1;
%          1 1 -1; -1 1 -1; -1 -1 -1; 1 -1 -1; 1 1 -1;
%          1 -1 -1; 1 -1 1; -1 -1 1; -1 -1 -1; -1 1 -1; -1 1 1];
% orEnvPlot(repmat(transpose(ab(:,1)),[size(edges,1) 1]) + repmat(transpose(ab(:,2)), [size(edges,1) 1]).*edges,...
%           'size',5,'color',[1 0 0],'line');

% intersect with object to see if anything hit
[collision, info] = orEnvRayCollision(diag([1 1 1 1000 1000 1000])*ApproachDirs, Target.id);

% make sure all normals are the correct sign: pointing outward from the object)
I = find(collision);
flip = dot(ApproachDirs(4:6,I),info(4:6,I))>0;
ApproachDirs = info(:,I);
ApproachDirs(4:6,find(flip)) = -ApproachDirs(4:6,find(flip));

if( add_spherenorms )
    %% approach from middle of sphere
    dirs = ApproachDirs(1:3,:) - repmat(ab(:,1),[1 size(ApproachDirs,2)]);
    ApproachDirs = [[ApproachDirs(1:3,:); dirs ./ repmat(sqrt(sum(dirs.^2)),[3 1])] ApproachDirs];
end

% plot the directions
orEnvClose();
orEnvPlot(transpose(ApproachDirs(1:3,:)), 'size',5,'color',[1 0 0]);
orEnvPlot(transpose(reshape([ApproachDirs(1:3,:); ApproachDirs(1:3,:)+0.005*ApproachDirs(4:6,:)],[3 size(ApproachDirs,2)*2])),...
          'size',4,'color',[1 0 0],'linelist');

counter = 0;
grasp = [];
showhandles = 1;
oldhandles = [];

%orEnvSetOptions('collision pqp');
totalgrasps = size(ApproachDirs,2)*size(preshapes,2)*length(rolls)*length(standoffs);
for approach = ApproachDirs
    for roll = rolls
        for preshape = preshapes
            for standoff = standoffs
                counter = counter+1;
                disp(['Generating example ' num2str(counter) ' of ' num2str(totalgrasps)]);

                grasp(robot.grasp.center) = approach(1:3);
                grasp(robot.grasp.direction) = -approach(4:6);
                grasp(robot.grasp.roll) = roll;
                grasp(robot.grasp.standoff) = standoff;
                grasp(robot.grasp.joints) = preshape;

                allvol = [];
                allmindist = [];
                center_offset_backup = grasp(robot.grasp.center);
                direction_backup = grasp(robot.grasp.direction);
                for q = 1:num_tries

                    if(use_noise && q > 1)
                        newdir = direction_backup + 0.2*rand(1,3) - 0.1*ones(1,3);
                        grasp(robot.grasp.direction) = newdir/norm(newdir);
                        grasp(robot.grasp.center) = center_offset_backup + 0.03*rand(1,3) - 0.015*ones(1,3);
                    end
                    
                    [contactsraw,Thand] = RunGrasp(robot,grasp,Target,0,friction_mu);
                    %disp(['Analyzing example ' num2str(counter)]);

                    if(size(contactsraw) == 0)
                        disp('No contact points for this example.');
                        allvol = [allvol 0];
                        allmindist = [allmindist 0];
                    else
                        contacts = sscanf(contactsraw,'%f');
                        mindist = contacts(end-1);
                        vol = contacts(end);
                        contacts = contacts(1:(end-2));
                        contacts = reshape(contacts, [6 length(contacts)/6]);

                        if( showhandles )
                            handles = DrawContacts(contacts,friction_mu);
                            if( ~isempty(oldhandles) )
                                orEnvClose(oldhandles);
                            end
                            oldhandles = handles;
                        end
                        
                        allvol = [allvol vol];
                        allmindist = [allmindist mindist];
                    end
                end
                vol = min(allvol);%mean(allvol,2);
                mindist = min(allmindist);%mean(allmindist,2);

                grasp(robot.grasp.center) = center_offset_backup;
                grasp(robot.grasp.direction) = direction_backup;
                grasp(robot.grasp.transform) = reshape(Thand(1:3,1:4),[1 12]);
                GraspTable = [GraspTable; grasp];
                GraspStats = [GraspStats; mindist vol allmindist allvol];

                if (mindist > 1e-9 )
                    disp(sprintf('grasp found %d, total good grasps: %d', counter, sum(GraspStats(:,1) > 0)));
                end
                
                if( mod(counter,100) == 99 )
                    save('-ascii','tempgrasptable.mat','GraspTable','GraspStats');
                end
            end
        end
    end
end
