% Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
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
addpath(fullfile(pwd,'octave'));
robotfile = 'robots/pa10manip.robot.xml'
gripperjoints = '0.03'
maskfile = fullfile(rosoct_findpackage('visibilitygrasping'),'pa10gripper_mask.mat');
convexfile = fullfile(rosoct_findpackage('visibilitygrasping'),'pa10gripper_convex.mat');
visibilityfile = fullfile(rosoct_findpackage('visibilitygrasping'),'cereal_visibility.mat');
visibility_fullfile = fullfile(rosoct_findpackage('visibilitygrasping'),'cereal_visibility_pa10.mat');
kinbodyfile = fullfile(rosoct_findpackage('visibilitygrasping'),'scenes','cereal_frootloops.kinbody.xml');

system(['./rvisionplanning.py --func=mask --rayoffset=0.03 --robotfile=' robotfile ' --gripperjoints="' gripperjoints '" --savefile=' maskfile])

Imask = load(maskfile);
[bestconvexhull,bestconvexpoints,bestvolume] = GetLargestFreeConvexPolygon(Imask,convexfile);

system(['./rvisionplanning.py --func=visibility --robotfile=' robotfile ' --kinbodyfile=' kinbodyfile ' --gripperjoints="' gripperjoints '" --savefile=' visibility_fullfile ' --convexfile=' convexfile ' --visibilityfile=' visibilityfile])
