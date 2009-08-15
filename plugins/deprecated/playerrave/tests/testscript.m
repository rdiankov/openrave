% Script used to test the synchornization and other advanced features of the player controller
% The specific robot used is the Barrett WAM with the Barrett Hand

robotid = 1;
wamhandfile = '../../openrave_local/robots/barrettwam.robot.xml';
handfile = 'robots/barretthand.robot.xml';
wamfile = 'robots/wam.robot.xml';
curdir = pwd;

%% testing the hand controller

orEnvLoadScene(handfile,1);
orRobotControllerSet(robotid,'playercontroller','wam  6665 actarray 0')
orEnvCreateProblem('basemanipulation','testhand');
orEnvSetOptions('debug 1');
orRobotSetDOFValues(1,0,0)
orProblemSendCommand(['traj ' curdir '/handtesttraj.txt']);
orProblemSendCommand(['traj ' curdir '/handtesttraj2.txt']);
orProblemSendCommand(['traj ' curdir '/handtesttraj3.txt']);
orProblemSendCommand(['traj ' curdir '/handtesttraj4.txt']);

orRobotControllerSend(robotid,'setvel 1 0 0 0'); % set velocity
orRobotControllerSend(robotid,'setvel -1 0 0 0'); % set velocity

orRobotControllerSend(robotid,'reset'); % set velocity

orRobotControllerSet(robotid,'playercontroller','wam  6664 actarray 0')
orProblemSendCommand(['traj ' curdir '/handtesttraj2.txt']);
orRobotSetDOFValues(1,pi/2,3);
orRobotSetDOFValues(1,0,3);

% brakes
orProblemSendCommand('traj handtesttraj3.txt');
orRobotControllerSend(robotid,'brake 1'); % turn off brakes
orRobotControllerSend(robotid,'brake 0'); % turn off brakes

%% test syncing between wam/hand
orEnvLoadScene(wamfile,1);
orRobotControllerSet(robotid,'playercontroller','wam  6665 actarray 0')

orEnvLoadScene(wamhandfile,1);
orRobotControllerSet(robotid,'playercontroller','wam  6665 actarray 0 actarray 1')
orEnvSetOptions('debug 1');
orEnvCreateProblem('basemanipulation','barrettwam');

newspeeds = [1.5708    1.0472    2.0944    2.0944 4.1888 4.1888 1.0472]*0.5;
for i = 1:length(newspeeds)
    orRobotControllerSend(robotid, ['setspeed ' num2str(i-1) ' ' num2str(newspeeds(i))]);
end

dof = orBodyGetDOF(robotid);
orProblemSendCommand(['MoveActiveJoints goal ' sprintf('%f ', zeros(1,dof))]);
orRobotSetDOFValues(1,zeros(1,dof),0:(dof-1));

% test 1 - simple traj sync
orProblemSendCommand(['traj ' curdir 'wam4testtraj.txt']);
orProblemSendCommand(['traj ' curdir 'wam4testtraj.txt']);

% test 2 - two velocities between trajectories
orProblemSendCommand(['traj ' curdir 'wamtesttraj.txt']);
orRobotControllerSend(robotid, 'ignoreproxy 0');
orRobotControllerSend(robotid,'setvel 0 0 0 0 1 0 0 0'); % set velocity
orRobotControllerSend(robotid, 'ignoreproxy');
orRobotControllerSend(robotid,'setvel 0 0 0 0 -1 0 0 0'); % set velocity
orProblemSendCommand(['traj ' curdir 'wamtesttraj.txt']);

