% sensors = orRobotGetAttachedSensors(robotid)
%
% sensors is a cell array describing the attached sensors of the robot
% Each cell is a struct with fields:
%   name - name of the attached sensor
%   link - zero-based index of link sensor is attached to
%   Trelative - 3x4 matrix of the relative transform of the camera with respect to the robot
%   Tglobal - 3x4 matrix of the global transform of the sensor of the current robot
%             Tglobal = Tlink * Trelative
%   type - the xml id of the sensor that is attached
function sensors = orRobotGetAttachedSensors(robotid)

out = orCommunicator(sprintf('robot_getsensors %d', robotid), 1);
sensors = {};

if(strcmp('error',sscanf(out,'%s',1)))
    error('Error getting manipulators');
end

newline = 10;
[tok, rem] = strtok(out, [' ' newline]);
numsensors = str2num(tok);
sensors = cell(numsensors,1);

for i = 1:numsensors
    [tok, rem] = strtok(rem, ' ');
    strnamelen = floor(str2num(tok));
    sensors{i}.name = rem(2:(strnamelen+1));
    rem = rem((strnamelen+2):end);
    
    [tok, rem] = strtok(rem, ' ');
    sensors{i}.link = str2num(tok);
    
    sensors{i}.Trelative = zeros(3,4);
    for j = 1:12
        [tok, rem] = strtok(rem, ' ');
        sensors{i}.Trelative(j) = str2num(tok);
    end

    [tok, rem] = strtok(rem, ' ');
    strtypelen = floor(str2num(tok));
    sensors{i}.type = rem(2:(strtypelen+1));
    rem = rem((strtypelen+2):end);
    
    sensors{i}.Tglobal = zeros(3,4);
    for j = 1:12
        [tok, rem] = strtok(rem, ' ');
        sensors{i}.Tglobal(j) = str2num(tok);
    end
end
