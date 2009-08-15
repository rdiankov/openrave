% demo to solve a hanoi puzzle with a Puma arm using OpenRAVE

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

more off
global probs
addopenravepaths_hanoi();

% initialize the scene
orEnvLoadScene('data/hanoi_complex.env.xml',1);
%orEnvLoadScene('data/hanoi.env.xml',1);
probs.manip = orEnvCreateProblem('basemanipulation','puma');
robot = RobotCreatePuma('puma');

% gather data about pegs and disks
disknames = {'disk0','disk1','disk2'};
heights = [0.021 0.062 0.103]+0.01;

disks = zeros([1 length(disknames)]);
diskradius = zeros([1 length(disknames)]);
for i = 1:length(disknames)
    disks(i) = orEnvGetBody(disknames{i});
    ab = orBodyGetAABB(disks(i));
    diskradius(i) = norm(ab(1,2))-0.02;
end

srcpeg = orEnvGetBody('srcpeg');
destpeg = orEnvGetBody('destpeg');
peg = orEnvGetBody('peg');

% initialize the hanoi puzzle lists
conf = {};
conf{srcpeg} = [1 2 3];
conf{destpeg} = [];
conf{peg} = [];

tic
hanoisolve(3, srcpeg, destpeg, peg, conf, robot, disknames, diskradius, heights);
t = toc
