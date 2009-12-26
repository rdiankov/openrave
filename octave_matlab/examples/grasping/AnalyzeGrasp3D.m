% [Volume, mindist] = AnalyzeGrasp3D(contacts)
%
% analyzes contact point in 3D for power grasps.
% if mindist is 0, grasp failed
% contacts is a 6xN array. The first 3 rows are the position and last 3 are
% the normals to the faces

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
function [mindist, Volume] = AnalyzeGrasp3D(contacts)
    
Volume = 0;
mindist = 0;
K = [];

%need at least 7 contact wrenches to have force closure in 3D
if(size(contacts,2) < 7)
    return;
end

% find the screw coordinates
S = [contacts(4:6,:); cross(contacts(1:3,:), contacts(4:6,:))];

try
    [K, Volume] = convhulln(transpose(S));
catch
    if( isempty(K) ) % octave needs this
        return;
    end
end

if( isempty(K) ) % octave needs this
    return;
end

Mean = mean(S,2);

mindist = Inf;

for i = 1:size(K,1)
    % check if 0 is in the center
    V = S(:,K(i,:));
    
    if( abs(det(V)) < 1e-15 )
        continue;
    end
    
    % V'n = ones
    n = transpose(V)\ones([6 1]);
    n = n / norm(n);

    dist = transpose(n)*V(:,1);
    meandist = transpose(n)*Mean;
    
    if( dist < meandist)
        dist = -dist;
    end
    
    if( dist < 0 || abs(dist-meandist) < 1e-15 )
        % failed due to center being outside of face or too close to face
        mindist = 0;
        return;
    end
    
    % for all faces, find the minimum dist
    mindist = min(mindist, dist);
end

if( isinf(mindist) )
    mindist = 0;
end
