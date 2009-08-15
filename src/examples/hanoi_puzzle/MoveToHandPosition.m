% success = MoveToHandPosition(T)
%
% moves to the hand position specified by the 3x4 matrix T
% success is 1 if succeed or 0 if it didn't

% by Rosen Diankov (rdiankov@cs.cmu.edu)
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
function success = MoveToHandPosition(T)
global probs

s = orProblemSendCommand(['movetohandposition trans ' sprintf('%f ',T(10:12)) ' rot ' sprintf('%f ',T(1:9))], probs.manip);
success = 0;
if( ~isempty(s) )
    success = str2num(s) > 0;
end

% code to perform smoothing in a separate step
% trajname = fullfile(pwd,'movetraj.txt');
% s = orProblemSendCommand(['movetohandposition execute 0 writetraj ' trajname ' trans ' sprintf('%f ',T(10:12)) ' rot ' sprintf('%f ',T(1:9))]);
% success = ~isempty(s) & str2num(s) > 0;
% if( ~success )
%     return;
% end
% 
% orProblemSendCommand(['SmoothTrajectory trajfile ' trajname]);
