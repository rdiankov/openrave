% bodies = orEnvGetBodies()
%
% bodies is a cell array of all body objects in the scene
% every cell contains a struct with the following parameters
% id - bodyid
% filename - filename used to initialize the body with
% name - human robot name
% type - xml type of body

function bodies = orEnvGetBodies()

out = orCommunicator('env_getbodies', 1);

if(strcmp('error',sscanf(out,'%s',1)))
    error('Error getting bodies');
end

newline = 10;
[tok, rem] = strtok(out, [' ' newline]);
numbodies = str2num(tok);
bodies = cell(numbodies,1);

for i = 1:numbodies
    [tok, rem] = strtok(rem, [' ' newline]);
    bodies{i}.id = str2num(tok);
    [bodies{i}.name, rem] = strtok(rem, ' ');
    [bodies{i}.type, rem] = strtok(rem, ' ');
    % read until end of line (since filename might contain whitespace)
    [tok, rem] = strtok(rem, newline);
    if( strcmp(tok, 'none') )
        bodies{i}.filename = '';
    else
        bodies{i}.filename = strtrim(tok);
    end
end
