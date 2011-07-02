% loadikfast(robotfile,manipname,iktype)
%
% Loads an IK solver on a robot's manipulator
% Arguments:
%  robotfile - openrave robot
%  manipname [optional] - if specified, only tests ik for those manipulators (zero-indexed)
function loadikfast(robotfile,manipname,iktype)

if( ~exist('robotfile','var') )
    robotfile = 'robots/barrettwam.robot.xml';
end
if( ~exist('manipname','var') )
    manipname = '';
end
if( ~exist('iktype','var') )
    iktype = 'transform6d';
end

orEnvLoadScene('',1);
robotid = orEnvCreateRobot('robot',robotfile);
probid = orEnvCreateProblem('ikfast');
orRobotSetActiveManipulator(robotid,manipname);
s = orProblemSendCommand(['LoadIKFastSolver robot ' iktype],probid);
