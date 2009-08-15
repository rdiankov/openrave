% robotid = orEnvCreateRobot(robotname, xmlfile, type)
%
% Creates a robot of the given type. If type is not specified, creates a generic robot

function robotid = orEnvCreateRobot(robotname, xmlfile, type)

if( ~exist('type', 'var') )
    type = 'GenericRobot';
end

out = orCommunicator(['createrobot ' robotname ' ' xmlfile ' ' type], 1);

%if(strcmp('error',sscanf(out,'%s',1)))
%    error('Error creating planner');
%end

robotid = str2double(out);