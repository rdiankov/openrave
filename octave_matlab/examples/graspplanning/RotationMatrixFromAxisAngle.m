%% R = RotationMatrixFromAxisAngle(axisangle)
%%
%% v - is a 3x1 matrix that is axis*angle
%% R - 3x3 rotation matrix

% Copyright (C) 2008-2009 Rosen Diankov
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
function R = RotationMatrixFromAxisAngle(axisangle)

angle = norm(axisangle);
if( angle > 0 )
    R = RotationMatrixFromQuat([cos(angle/2); axisangle/angle*sin(angle/2)]);
else
    R = eye(3);
end
