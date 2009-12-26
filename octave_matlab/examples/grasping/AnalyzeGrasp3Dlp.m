% mindist = AnalyzeGrasp3Dlp(contacts)
%
% linear programming way of solving for force closure (similar to AnalyzeGrasp3D)

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
function mindist = AnalyzeGrasp3Dlp(contacts)
    
mindist = 0;
K = [];

% find the screw coordinates
S = [contacts(4:6,:); cross(contacts(1:3,:), contacts(4:6,:))];

%need at least 7 contact wrenches to have force closure in 3D
if(size(S,2) < 7 || rank(S) < 6)
    return;
end

wc = mean(S,2);
T = S - repmat(wc, [1 size(S,2)]);

if( isoctave() )
    ctype = repmat('U',[1 size(T,2)]);
    [x,mindist,exitflag] = glpk(wc,transpose(T),ones(size(S,2),1),[],[],ctype);
all((transpose(S)*x<=1.000001*ones(size(S,2),1)))
x
mindist
exitflag
    testdone = 180; % looking for optimal solution
else
    [x,mindist,exitflag,output,lambda] = linprog(wc,transpose(T),ones(size(S,2),1),[],[],[],[],[],optimset('Display','off'));
    testdone = 1;
end

mindist = -mindist;

if (mindist <0) || (mindist>1) || (exitflag ~=testdone)
    mindist = -1;
end
