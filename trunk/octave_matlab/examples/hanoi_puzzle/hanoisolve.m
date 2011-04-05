% conf = hanoisolve(n, from, to, by, oldconf, robot, disknames, diskradius, heights)
%
% recursive function to solve the hanoi problem

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
function conf = hanoisolve(n, from, to, by, oldconf, robot, disknames, diskradius, heights)

conf = oldconf;

if( n == 1 )
    % move the disk
    disk = conf{from}(end);
    disp(sprintf('hanoimove disk %d from %d to %d', disk, from, to));
    if( hanoimove(robot, disknames{disk}, diskradius(disk), from, to, heights(length(conf{to})+1)) == 0 )
        error('failed to solve hanoi');
    end
    % add the disk onto the correct peg list
    conf{to}(end+1) = disk;
    conf{from}(end) = [];
else
    conf = hanoisolve(n-1, from, by, to, conf, robot, disknames, diskradius, heights);
    conf = hanoisolve(1, from, to, by, conf, robot, disknames, diskradius, heights);
    conf = hanoisolve(n-1, by, to, from, conf, robot, disknames, diskradius, heights);
end
