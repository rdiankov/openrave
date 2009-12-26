% Gpruned = PruneGraspSet(G, nsize, thresh)
%
% Prunes region of the grasp set that are too close
% nsize - final grasp set size
% thresh - pruning threshold (higher reduces grasp set)

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
