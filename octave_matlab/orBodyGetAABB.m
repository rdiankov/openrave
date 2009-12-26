% aabb = orBodyGetAABB(bodyid)
%
% returns an axis-aligned boudning box of the body in world coordinates
% aabb is a 3x2 vector where the first column is the position of the box
% and the second is the extents

function aabb = orBodyGetAABB(bodyid)

command_str = ['body_getaabb ' num2str(bodyid)];

out = orCommunicator(command_str, 1);

if(strcmp('error',sscanf(out,'%s',1)))
    error('Error orBodyGetAABB');
end

aabb = reshape(sscanf(out,'%f'),3,2);