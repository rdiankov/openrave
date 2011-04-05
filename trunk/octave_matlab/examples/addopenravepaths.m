function addopenravepaths()

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
    addpath(fullfile(pwd,'..')); % can possibly get from 
else
    addpath(fullfile(basepath,'share','openrave',langname));
end
