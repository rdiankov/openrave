% aabbs = orBodyGetAABBs(bodyid)
%
% returns the axis-aligned boudning boxes of all the links of the body in world coordinates
% aabbs is a 6xn vector where each column describes the box for all n links.
% The first 3 values in each column describe the position of the aabb, and the next
% 3 values describe the extents (half width/length/height) on each of the axes.

function aabbs = orBodyGetAABBs(bodyid)

command_str = ['body_getaabbs ' num2str(bodyid)];

out = orCommunicator(command_str, 1);

if(strcmp('error',sscanf(out,'%s',1)))
    error('Error orBodyGetAABBs');
end

aabbs = sscanf(out,'%f');
aabbs = reshape(aabbs,6,length(aabbs)/6);
