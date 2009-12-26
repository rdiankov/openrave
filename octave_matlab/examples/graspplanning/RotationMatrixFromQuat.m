% R = RotationMatrixFromQuat(quat)
%
% quat - the format is [cos(angle/2) axis*sin(angle/2)]

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
function R = RotationMatrixFromQuat(quat)

R = zeros(3,3);
qq1 = 2*quat(2)*quat(2);
qq2 = 2*quat(3)*quat(3);
qq3 = 2*quat(4)*quat(4);
R(1,1) = 1 - qq2 - qq3;
R(1,2) = 2*(quat(2)*quat(3) - quat(1)*quat(4));
R(1,3) = 2*(quat(2)*quat(4) + quat(1)*quat(3));
R(2,1) = 2*(quat(2)*quat(3) + quat(1)*quat(4));
R(2,2) = 1 - qq1 - qq3;
R(2,3) = 2*(quat(3)*quat(4) - quat(1)*quat(2));
R(3,1) = 2*(quat(2)*quat(4) - quat(1)*quat(3));
R(3,2) = 2*(quat(3)*quat(4) + quat(1)*quat(2));
R(3,3) = 1 - qq1 - qq2;
