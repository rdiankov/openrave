% trajectory = orPlannerPlan(planner)
%
% Start planning. The planner returns a trajectory when successful (otherwise returns an empty matrix)
% The trajectory is an (DOF+1)xN matrix where N is the number of points in the trajectory. The first row
% are the time values of each trajectory point.


function trajectory = orPlannerPlan(planner)

out = orCommunicator(['planner_plan ' num2str(planner)], 1);

if(strcmp('error',sscanf(out,'%s',1)))
    error('Error creating planner');
end

out = str2num(out)';

trajectory = reshape(out(3:end),out(1)+1,out(2));