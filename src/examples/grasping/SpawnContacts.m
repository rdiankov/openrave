% C = SpawnContacts(contacts, mu, N)
%
% given contact normals and a friction coefficient, spawns normal
% vectors around the surface of the friction cone
% contacts - 6xN matrix of contact points/normals
% mu - friction coefficient
% N - how many vectors to approximate the friction cone by
% C - 6xN matrix of contact points/normals on friction cones

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
function C = SpawnContacts(contacts, mu, N)

if(size(contacts,1) == 0)
    C= [];
    return;
end

if(mu == 0)
    C = contacts;
    return
end

C = [];
for i = 1:size(contacts,2)
    Norms = getFrictionCone(contacts(4:6,i), mu, N);
    
    C = [C [repmat(contacts(1:3,i), [1 N]); Norms]];
end

function C = getFrictionCone(normal, mu, N)

right = [1;0;0]+0.2*rand([3 1]);
right = right - normal * transpose(normal) * right;
right = right / norm(right);
up = cross(right,normal);

angles = 0:(2*pi/N):2*pi;
angles = angles(1:N);

C = repmat(normal, [1 N]) + mu * (right * cos(angles) + up * sin(angles));
C = C ./ repmat(sqrt(sum(C.^2)), [3 1]);
