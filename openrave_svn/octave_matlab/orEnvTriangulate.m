% [tripoints, triindices] = orEnvTriangulate(inclusive, ids)
%
% Returns the triangulation of various objects in the scenes controlled by name and options
% Arguments:
%   inclusive - if 1, will only triangulate the bodies pointed to by ids
%               if 0, will triangulate all objects except the bodies pointed to by ids
%               default value is 0.
%   ids (optional) - the ids to include or exclude in the triangulation
% To triangulate everything, just do orEnvTriangulate(0,[]), or orEnvTriangulate()
%
% Output:
%   tripoints - 3xN matrix of 3D points
%   tripoints - 3xK matrix of indices into tripoints for every triangle.
function [tripoints, triindices] = orEnvTriangulate(inclusive, ids)

if( ~exist('inclusive','var') )
    inclusive = 0;
end
if( ~exist('ids','var') )
  ids = [];
end

out = orCommunicator(['env_triangulate ' num2str(inclusive) ' ' sprintf('%d ', ids)], 1);

values = sscanf(out,'%f');
pointsend = values(1)*3+2;
if( values(1) > 0 )
    tripoints = reshape(values(3:pointsend),[3 values(1)]);
else
    tripoints = [];
end
if( values(2) > 0 )
    triindices = reshape(values((pointsend+1):end),[3 values(2)])+1;
else
    triindices = [];
end

