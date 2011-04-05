%% [trajvals,transvals,timevals] = ParseTrajData(trajdata)
%%
%% parses a trajectory file 

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
function [trajvals,transvals,timevals] = ParseTrajData(trajdata)
vals = sscanf(trajdata, '%f');
numpts = vals(1);
numdof = vals(2);

newvals = reshape(vals(4:(3+numpts*(numdof+8))),[numdof+8 numpts]);
timevals = newvals(1,:);
transvals = newvals((2+numdof):end,:);
trajvals = newvals(2:(1+numdof),:);
