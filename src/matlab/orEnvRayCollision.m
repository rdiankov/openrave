% [collision, colinfo] = orEnvRayCollision(rays,[bodyid])
%
% performs ray collision checks and returns the position and normals
% where all the rays collide
% rays - a 6xN matrix where the first 3
% rows are the ray position and last 3 are the ray direction
% collision - N dim vector that is 1 for colliding rays and 0
% for non-colliding rays colinfo is a 6xN vector that describes 
% where the ray hit and the normal to the surface of the hit point
% where the first 3 columns are position and last 3 are normals
% if bodyid is specified, only checks collisions with that body

function [collision, colinfo] = orEnvRayCollision(varargin)

rays = varargin{1};
    
if(size(rays,1) ~= 6)
   error('rays must be a 6xN matrix');
end

numrays = size(rays,2);
cmdstr = ['env_raycollision '];

if(size(varargin,2)  == 2)
    bodyid = varargin{2};
    if(~isscalar(bodyid))
        error('can only specify one bodyid');
    end
    cmdstr = [cmdstr ' ' num2str(bodyid) ' '];
else
    cmdstr = [cmdstr ' -1 '];
end

cmdstr = [cmdstr, sprintf('%f ', rays(:))];
out = orCommunicator(cmdstr, 1);

if(strcmp('error',sscanf(out,'%s',1)))
    collision = [];
    colinfo = [];
else
    out = sscanf(out,'%f')';
    
    collision = out(1:numrays);
    colinfo = reshape(out(numrays+1:end),6,numrays);
end
