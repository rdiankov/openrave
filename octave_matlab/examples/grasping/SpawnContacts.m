% C = SpawnContacts(contacts, mu, N)
%
% given contact normals and a friction coefficient, spawns normal
% vectors around the surface of the friction cone
% contacts - 6xN matrix of contact points/normals
% mu - friction coefficient
% N - how many vectors to approximate the friction cone by
% C - 6xN matrix of contact points/normals on friction cones

% Copyright (C) 2008-2010 Rosen Diankov, Dmitry Berenson
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
