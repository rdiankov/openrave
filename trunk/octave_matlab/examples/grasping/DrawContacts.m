% handles = DrawContacts(contacts, friction, conelength, transparency)
%
% draws the contact cones with a specific friction in OpenRAVE
% contacts - 6xN vector of N contacts. First 3 rows are position, last 3 are normals
% friction - coefficient friction, tan of cone angle
% length (optional) - how long to make the cones when drawing, default is 0.04
% transparency (optional) - the transparency level of the cones, default is 0.5
% handles - returns the OpenRAVE handles

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
function handles = DrawContacts(contacts, friction, conelength, transparency)

if( ~exist('conelength','var') )
    conelength = 0.02;
end
if( ~exist('transparency','var') )
    transparency = 0.5;
end

conesegs = 10;
conepoints = [0;0;0];
angs = 2*pi*(0:(conesegs-1))/conesegs;
conepoints = [[0;0;0] conelength*[friction*cos(angs);friction*sin(angs);ones(1,conesegs)]];
triinds = 1+[zeros(1,conesegs); 2:conesegs 1; 1:conesegs];

allpoints = [];

for i = 1:size(contacts,2)
    rotaxis = cross([0;0;1],contacts(4:6,i));
    sinang = norm(rotaxis);
    if( sinang > 0.001 )
        R = rodrigues(rotaxis/sinang*atan2(sinang,contacts(6,i)));
    else
        R = eye(3);
        R([5 9]) = sign(contacts(6,i));
    end
    
    points = R * conepoints + repmat(contacts(1:3,i),[1 size(conepoints,2)]);
    allpoints = [allpoints points(:,triinds(:))];
end

handles = orEnvPlot(transpose(allpoints), 'color',[1 0.4 0.4],'transparency',transparency,'trilist');
