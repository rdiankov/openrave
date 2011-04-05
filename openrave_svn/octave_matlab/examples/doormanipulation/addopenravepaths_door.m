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
function addopenravepaths_door()

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
    addpath(fullfile(pwd,'..','..')); % can possibly get from 
else
    addpath(fullfile(basepath,'share','openrave',langname));
end
