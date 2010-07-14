% demo to solve a hanoi puzzle with a Puma arm using OpenRAVE

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
more off
global probs
addopenravepaths_hanoi();

% initialize the scene
orEnvLoadScene('data/hanoi_complex.env.xml',1);
%orEnvLoadScene('data/hanoi.env.xml',1);
probs.manip = orEnvCreateProblem('basemanipulation','Puma');
probs.ikfast = orEnvCreateProblem('ikfast');
probs.robotname = 'Puma';
robot = RobotCreatePuma('Puma');

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
