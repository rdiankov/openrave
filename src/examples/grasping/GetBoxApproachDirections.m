% ApproachDirs = GetBoxApproachDirections(ab,step)
%
% finds the approach directions given a uniform box sampling around the object of interest
% center - 3x1 matrix for center of box
% extents - 3x1 matrix for extents of box
% step - stepsize between samples on surface of box
% ApproachDirs is a 6xN vector where the first 3 rows are the position and last 3 are direction

% Copyright (C) 2008 Rosen Diankov (rdiankov@cs.cmu.edu)
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
