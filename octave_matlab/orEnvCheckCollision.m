% [collision, colbodyid] = orEnvCheckCollision(bodyid,excludeid)
%
% Check collision of the robot with the environment. collision is 1 if the robot
% is colliding, colbodyid is the id of the object that body collided with

function [collision, colbodyid, contacts] = orEnvCheckCollision(bodyid,excludeid,getcontacts)


if( ~exist('excludeid', 'var') )
    excludeid = [];
end
if( ~exist('getcontacts', 'var') )
    getcontacts = 0;
end

out = orCommunicator(['body_checkcollision ' num2str(bodyid) ' ' num2str(length(excludeid)) ' ' sprintf('%d ',excludeid) ' ' num2str(getcontacts)], 1);

if(strcmp('error',sscanf(out,'%s',1)))
    error('Error checking collision');
end

data = sscanf(out, '%f');
collision = int32(data(1));
colbodyid = int32(data(2));

contacts = [];
if( getcontacts )
    N=(length(data)-2)/7;
    contacts = reshape(data(3:end),[7,N]);
end
