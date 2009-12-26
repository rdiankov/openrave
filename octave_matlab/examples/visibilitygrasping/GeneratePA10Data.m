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
