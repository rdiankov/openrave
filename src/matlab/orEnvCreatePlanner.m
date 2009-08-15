% plannerid = orEnvCreatePlanner(plannertype)

function plannerid = orEnvCreatePlanner(plannertype)

out = orCommunicator(['createplanner ' plannertype], 1);

%if(strcmp('error',sscanf(out,'%s',1)))
%    error('Error creating planner');
%end

plannerid = str2double(out);