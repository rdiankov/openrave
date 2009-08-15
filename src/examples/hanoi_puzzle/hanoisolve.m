% conf = hanoisolve(n, from, to, by, oldconf, robot, disknames, diskradius, heights)
%
% recursive function to solve the hanoi problem

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
