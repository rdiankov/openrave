% testik(robotfile,manipname,ikfastlib)
%
% tests the inverse kinematics solver of a robot
% Arguments:
%  robotfile - openrave robot
%  manipname [optional] - if specified, only tests ik for that manipulator
%  rotonly [optional] - if specified and true, only test rotation component of ik solution
%  ikfastlib [optional] - the ikfast shared object to dynamically load as an openrave iksolver
function testik(robotfile,manipname)

more off; % turn off output paging
addopenravepaths()

if( ~exist('robotfile','var') )
    robotfile = 'robots/barrettwam.robot.xml';
end
if( ~exist('manipname','var') )
    manipname = '';
end

if( length(robotfile) > 0)
    orEnvLoadScene('',1);
    robotid = orEnvCreateRobot('robot',robotfile);
end

probid = orEnvCreateProblem('ikfast');
s = orProblemSendCommand(['LoadIKFastSolver robot Transform6D'],probid)

orRobotSetActiveManipulator(robotid,manipname);
manips = orRobotGetManipulators(robotid);

manipid = 1;
if( length(manipname) > 0)
    for i = 1:length(manips)
        if( strcmp(manips{i}.name, manipname) )
            manipid = i;
            break;
        end
    end
end

%% test any specific ik configuration
armdof = length(manips{1}.armjoints);
testvalues = [ 0.919065 -1.4331 1.45619 1.31858 0.696941 1.52955 -0.314613, 0, 0];

orBodySetJointValues(robotid,testvalues(1:armdof),manips{1}.armjoints);
links = orBodyGetLinks(robotid);
Thand = reshape(links(:,manips{1}.eelink+1),[3 4]);
Tee = [Thand; 0 0 0 1]*[manips{1}.Tgrasp; 0 0 0 1]
%Thand_frombase = inv([reshape(links(:,manips{1}.baselink+1),[3 4]);0 0 0 1]) * [Thand; 0 0 0 1];
s = orProblemSendCommand(['IKTest robot robot matrix ' sprintf('%f ',Tee(1:3,1:4))],probid);
s
if( isempty(s) )
    return;
end
orBodySetJointValues(robotid,sscanf(s,'%f'),manips{1}.armjoints);

%% can also do this through the ik param type:
quat = QuatFromRotationMatrix(Tee(1:3,1:3))
s = orProblemSendCommand(['IKTest robot robot ikparam  1728053249 ' sprintf('%f ',[quat(:);Tee(1:3,4)])],probid);

%% if ik solver supports translation 3d, can also call its ik using 0x33000003
%s = orProblemSendCommand(['IKTest robot robot ikparam  ' sprintf('%d ',0x33000003) ' 0.1 0.2 0.3'])

disp('now testing ik')
cmd = 'debugik numtests 100 robot robot ';
out=orProblemSendCommand(cmd,probid);
res=sscanf(out,'%f');
disp(['success rate ' sprintf('%f',res(2)/res(1))])
