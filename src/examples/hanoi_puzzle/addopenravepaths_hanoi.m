function addopenravepaths_hanoi()

if( exist('orEnvSetOptions','file') )
   %% functions already added
   return;
end

basepath = [];
if( exist('OCTAVE_VERSION') ~= 0 )
    langname = 'octave';
else
    langname = 'matlab';
end

if( isunix() )
    %% try getting from openrave-config
    [status,basepath] = system('openrave-config --prefix');
    basepath = strtrim(basepath);
end

if( isempty(basepath) )
    %% couldn't find so guess
    basepath = fullfile(pwd,'..','..','..','..'); % can possibly get from 
end

addpath(fullfile(basepath,'share','openrave',langname));
