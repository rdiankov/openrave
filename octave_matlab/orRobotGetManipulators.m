% manipulators = orRobotGetManipulators(robotid)
%
% manipulators is a cell array describes the manipulators of the robot
% Each cell is a struct with fields
%   baselink - zero-based index of base link manipulator is attached to
%   eelink - zero-based index of link defining the end-effector
%   Tgrasp - 3x4 matrix of the grasp frame relative to the end effector link,
%            Tglobalgrasp = Tendeffector*Tgrasp
%   joints - 1xK zero-based joint indices of the hand attached to the end effector
%   armjoints - 1xN zero-based manipulator joint indices that have an
%               effect on the end effector
%   iksolvername - name of ik solver to use
function manipulators = orRobotGetManipulators(robotid)

out = orCommunicator(sprintf('robot_getmanipulators %d', robotid), 1);
manipulators = {};

if(strcmp('error',sscanf(out,'%s',1)))
    error('Error getting manipulators');
end

newline = 10;
[tok, rem] = strtok(out, [' ' newline]);
nummanips = str2num(tok);
manipulators = cell(nummanips,1);

for i = 1:nummanips
    [tok, rem] = strtok(rem, ' ');
    manipulators{i}.baselink = str2num(tok);
    [tok, rem] = strtok(rem, ' ');
    manipulators{i}.eelink = str2num(tok);
    
    manipulators{i}.Tgrasp = zeros(3,4);
    for j = 1:12
        [tok, rem] = strtok(rem, ' ');
        manipulators{i}.Tgrasp(j) = str2num(tok);
    end

    [tok, rem] = strtok(rem, ' ');
    numjoints = str2num(tok);
    manipulators{i}.joints = zeros(1,numjoints);
    for j = 1:numjoints
        [tok, rem] = strtok(rem, ' ');
        manipulators{i}.joints(j) = str2num(tok);
    end
    manipulators{i}.handjoints = manipulators{i}.joints;

    [tok, rem] = strtok(rem, ' ');
    numarmjoints = str2num(tok);
    manipulators{i}.armjoints = zeros(1,numarmjoints);
    for j = 1:numarmjoints
        [tok, rem] = strtok(rem, ' ');
        manipulators{i}.armjoints(j) = str2num(tok);
    end

    [tok, rem] = strtok(rem, ' ');
    numclosing = str2num(tok);
    manipulators{i}.closingdir = zeros(1,numclosing);
    for j = 1:numclosing
        [tok, rem] = strtok(rem, ' ');
        manipulators{i}.closingdir(j) = str2num(tok);
    end

    manipulators{i}.palmdir = zeros(3,1);
    for j = 1:3
        [tok, rem] = strtok(rem, ' ');
        manipulators{i}.palmdir(j) = str2num(tok);
    end

    [tok, rem] = strtok(rem, ' ');
    striklen = floor(str2num(tok));
    manipulators{i}.name = rem(2:(striklen+1));
    rem = rem((striklen+2):end);

    [tok, rem] = strtok(rem, ' ');
    striklen = floor(str2num(tok));
    manipulators{i}.iksolvername = rem(2:(striklen+1));
    rem = rem((striklen+2):end);
end
