% success = MoveToHandPosition(T)
%
% moves to the hand position specified by the 3x4 matrix T
% success is 1 if succeed or 0 if it didn't

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
function success = MoveToHandPosition(T)
global probs

s = orProblemSendCommand(['movetohandposition matrix ' sprintf('%f ',T(1:12))], probs.manip);
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
