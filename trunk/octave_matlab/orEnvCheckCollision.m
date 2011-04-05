% [collision, colbodyid,contacts] = orEnvCheckCollision(bodyid,excludeid,getcontacts,linkindex)
%
% Check collision of the robot with the environment. collision is 1 if the robot
% is colliding, colbodyid is the id of the object that body collided with
%% bodyid - id of the body
%% excludeid - list of ids to exclude from collision
%% getcontacts - if 1 then returns N contacts as a 7xN matrix
%% linkindex

function [collision, colbodyid, contacts] = orEnvCheckCollision(bodyid,excludeid,getcontacts,linkindex)


if( ~exist('excludeid', 'var') )
    excludeid = [];
end
if( ~exist('getcontacts', 'var') )
    getcontacts = 0;
end
if( ~exist('linkindex', 'var') )
    linkindex = -1;
end

out = orCommunicator(['body_checkcollision ' num2str(bodyid) ' ' num2str(length(excludeid)) ' ' sprintf('%d ',excludeid) ' ' num2str(getcontacts) ' ' num2str(linkindex)], 1);

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
