function [name,id] = FindTarget(pattern)
name = [];
id = [];
bodies = orEnvGetBodies(0,openraveros_BodyInfo().Req_Names());
for i = 1:length(bodies)
    if( regexp(bodies{i}.name, pattern) )
        name = bodies{i}.name;
        id = bodies{i}.id;
        break;
    end
end
