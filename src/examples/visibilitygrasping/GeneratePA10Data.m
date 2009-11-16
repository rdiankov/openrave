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
