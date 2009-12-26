% Gpruned = PrunePoints(G, nsize, thresh2,neighsize,ispose)
%
% Prunes region of the grasp set that are too close
% nsize - final grasp set size
% thresh2 - pruning threshold squared (higher reduces grasp set)

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
function Gpruned = PrunePoints(G, nsize, thresh2,neighsize,ispose)

if( ~exist('neighsize','var') )
    neighsize = 1;
end

if( ~exist('ispose','var') )
    ispose = 1;
end

if( ispose )
    W = [0.2;0.2;0.2;0.2;1;1;1];
else
    W = ones(size(G,1),1);
end

iter = 1;
while(size(G,2)>nsize)
    ind = ceil(size(G,2).*rand);
    g = G(:,ind);
    d = W'*(G - repmat(g, [1 size(G,2)])).^2;

    if( ispose )
        g2 = [-g(1:4);g(5:7)];
        % check g's neighbors
        d2 = W'*(G - repmat(g2, [1 size(G,2)])).^2;
        neigh = sum(d < thresh2 | d2 < thresh2);
    else
        neigh = sum(d < thresh2);
    end

    if( neigh > neighsize )
        G(:,ind) = [];
        %size(G,2)
    end
    
    iter = iter+1;
    if( iter > 5000 )
        break;
    end
end

length(G)
Gpruned = G;
