% handles = DrawContacts(contacts, friction, conelength, transparency)
%
% draws the contact cones with a specific friction in OpenRAVE
% contacts - 6xN vector of N contacts. First 3 rows are position, last 3 are normals
% friction - coefficient friction, tan of cone angle
% length (optional) - how long to make the cones when drawing, default is 0.04
% transparency (optional) - the transparency level of the cones, default is 0.5
% handles - returns the OpenRAVE handles

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
