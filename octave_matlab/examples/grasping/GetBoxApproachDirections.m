% ApproachDirs = GetBoxApproachDirections(ab,step)
%
% finds the approach directions given a uniform box sampling around the object of interest
% center - 3x1 matrix for center of box
% extents - 3x1 matrix for extents of box
% step - stepsize between samples on surface of box
% ApproachDirs is a 6xN vector where the first 3 rows are the position and last 3 are direction

% Copyright (C) 2008-2010 Rosen Diankov
% 
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
function ApproachDirs = GetBoxApproachDirections(center, extents, step)

% have 6 sides
sides = [0 0 extents(3) 0 0 -1 extents(1) 0 0 0 extents(2) 0;
         0 0 -extents(3) 0 0 1 extents(1) 0 0 0 extents(2) 0;
         0 extents(2) 0 0 -1 0 extents(1) 0 0 0 0 extents(3);
         0 -extents(2) 0 0 1 0 extents(1) 0 0 0 0 extents(3);
         extents(1) 0 0 -1 0 0 0 extents(2) 0 0 0 extents(3);
         -extents(1) 0 0 1 0 0 0 extents(2) 0 0 0 extents(3)];

ApproachDirs = [];

for side = transpose(sides)
    ex = norm(side(7:9));
    ey = norm(side(10:12));
    %% always add [0,0]
    [XX,YY] = meshgrid([-ex:step:-step/4 0 step:step:ex],[-ey:step:-step/4 0 step:step:ey]);
    localpos = side(7:9)*transpose(XX(:))/ex + side(10:12)*transpose(YY(:))/ey;
    ApproachDirs = [ApproachDirs [repmat(center+side(1:3),[1 size(localpos,2)])+localpos; repmat(side(4:6),[1 size(localpos,2)])]];
end
