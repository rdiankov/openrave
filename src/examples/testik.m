% testrosik(robotfile,manipid,rotonly,ikfastlib)
%
% tests the inverse kinematics solver of a robot
% Arguments:
%  robotfile - openrave robot
%  manipid [optional] - if specified, only tests ik for those manipulators (zero-indexed)
%  rotonly [optional] - if specified and true, only test rotation component of ik solution
%  ikfastlib [optional] - the ikfast shared object to dynamically load as an openrave iksolver
function testik(robotfile,manipid,rotonly,ikfastlib)

more off; % turn off output paging
addopenravepaths()

if( ~exist('robotfile','var') )
    robotfile = 'robots/barrettwam.robot.xml';
end
if( ~exist('rotonly','var') )
    rotonly = 0;
end

orEnvLoadScene('',1);
robotid = orEnvCreateRobot('robot',robotfile);
if( exist('ikfastlib','var') )
    manips = orRobotGetManipulators(robotid);
    ikfastprobid = orEnvCreateProblem ('IKFast');
    orProblemSendCommand (['AddIkLibrary ' manips{manipid+1}.iksolvername ' ' ikfastlib],ikfastprobid);
    %% have to reload robot
    orBodyDestroy(robotid);
    robotid = orEnvCreateRobot('robot',robotfile);
end
probid = orEnvCreateProblem('basemanipulation','robot');

manips = orRobotGetManipulators(robotid);

cmd = 'debugik numtests 200 ';
if(rotonly)
    cmd = [cmd ' rotonly '];
end

if( ~exist('manipid','var') )
    for i = 1:length(manips)
        orProblemSendCommand(sprintf('SetActiveManip %d',i-1),probid);
        tic;
        orProblemSendCommand(cmd,probid);
        toc
    end
else
    orProblemSendCommand(sprintf('SetActiveManip %d',manipid),probid);
    tic;
    orProblemSendCommand(cmd,probid);
    toc
end


%% test any specific ik configuration
% orBodySetJointValues(robotid,[ 0.919065 -1.4331 1.45619 1.31858 0.696941 1.52955 -0.314613],manips{1}.armjoints);
% links = orBodyGetLinks(robotid);
% Thand = reshape(links(:,manips{1}.eelink+1),[3 4]);
% Thand_frombase = inv([reshape(links(:,manips{1}.baselink+1),[3 4]);0 0 0 1]) * [Thand; 0 0 0 1];
% s = orProblemSendCommand(['iktest matrix ' sprintf('%f ',Thand(:))]);
% s
% if( isempty(s) )
%     return;
% end
% orBodySetJointValues(robotid,sscanf(s,'%f'),manips{1}.armjoints);