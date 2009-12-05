% [Volume, mindist] = AnalyzeGrasp3D(contacts)
%
% analyzes contact point in 3D for power grasps.
% if mindist is 0, grasp failed
% contacts is a 6xN array. The first 3 rows are the position and last 3 are
% the normals to the faces

% Copyright (C) 2008 Rosen Diankov (rdiankov@cs.cmu.edu), Dmitry Berenson
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
