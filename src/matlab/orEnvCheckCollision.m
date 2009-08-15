% [collision, colbodyid] = orEnvCheckCollision(bodyid,excludeid)
%
% Check collision of the robot with the environment. collision is 1 if the robot
% is colliding, colbodyid is the id of the object that body collided with

function [collision, colbodyid] = orEnvCheckCollision(bodyid,excludeid)


if( ~exist('excludeid', 'var') )
    excludeid = 0;
end

out = orCommunicator(['body_checkcollision ' num2str(bodyid) ' ' num2str(excludeid)], 1);

if(strcmp('error',sscanf(out,'%s',1)))
    error('Error checking collision');
end

data = sscanf(out, '%d');
collision = data(1);
colbodyid = data(2);


