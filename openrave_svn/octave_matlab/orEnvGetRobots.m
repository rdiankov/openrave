% robots = orEnvGetRobots()
%
% robots is a cell array of robots
% every cell contains a struct with the following parameters
% id - robotid
% filename - filename used to initialize the body with
% name - human robot name
% type - type of robot

function robots = orEnvGetRobots()

out = orCommunicator('env_getrobots', 1);

if(strcmp('error',sscanf(out,'%s',1)))
    error('Error getting robots');
end

newline = 10;
[tok, rem] = strtok(out, [' ' newline]);
numrobots = str2num(tok);
robots = cell(numrobots,1);

for i = 1:numrobots
    [tok, rem] = strtok(rem, [' ' newline]);
    robots{i}.id = str2num(tok);
    robots{i}.name = '';
    robots{i}.type = '';
    [robots{i}.name, rem] = strtok(rem, ' ');
    [robots{i}.type, rem] = strtok(rem, ' ');
    % read until end of line (since filename might contain whitespace)
    [tok, rem] = strtok(rem, newline);
    if( strcmp(tok, 'none') )
        robots{i}.filename = '';
    else
        robots{i}.filename = strtrim(tok);
    end
end
