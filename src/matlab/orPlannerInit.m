% success = orPlannerInit(planner, robot, parameters)
%
% Initialize a planner to plan for a robot and give some parameters

function success = orPlannerInit(planner, robot, parameters)

command_str = ['planner_init ' num2str(robot)];

if( exist('parameters', 'var') )
    if(ischar(parameters))
        command_str = [command_str, parameters];
    else
        error('parameters must be a string');
    end
end


out = orCommunicator(command_str, 1);

%if(strcmp('error',sscanf(out,'%s',1)))
%    error('Error initializing planner');
%end

success = str2double(out);