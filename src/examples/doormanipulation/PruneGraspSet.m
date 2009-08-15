% Gpruned = PruneGraspSet(G, nsize, thresh)
%
% Prunes region of the grasp set that are too close
% nsize - final grasp set size
% thresh - pruning threshold (higher reduces grasp set)

% Copyright (C) 2006-2008 Rosen Diankov (rdiankov@cs.cmu.edu)
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http:%www.gnu.org/licenses/>.
%
% Functions to plan with caging grasps. See
% Rosen Diankov, Siddhartha Srinivasa, Dave Ferguson, James Kuffner.
% Manipulation Planning with Caging Grasps. IEEE-RAS Intl. Conf. on Humanoid Robots, December 2008.
function Gpruned = PruneGraspSet(G, nsize, thresh)

W = [0.4;0.4;0.4;0.4;1;1;1];

iter = 1;
while(size(G,2)>nsize)
    ind = ceil(size(G,2).*rand);
    g = G(:,ind);
    % check g's neighbors
    D = G - repmat(g, [1 size(G,2)]);
    d = W'*D.^2;
    neigh = sum(d < thresh);
    if( neigh > 7 )
        G(:,ind) = [];
        %size(G,2)
    end
    
    iter = iter+1;
    if( iter > 20000 )
        break;
    end
end

length(G)
Gpruned = G;
