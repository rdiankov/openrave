% quat = QuatFromRotationMatrix(R)
%
% R - 3x3 orthogonal rotation matrix
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
function quat = QuatFromRotationMatrix(R)

quat = zeros(4,1);
tr = R(1,1) + R(2,2) + R(3,3);
if (tr >= 0)
    quat(1) = tr + 1;
    quat(2:4) = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1) - R(1,2)];
else
    %% find the largest diagonal element and jump to the appropriate case
    [rmax, convcase] = max([R(1,1) R(2,2) R(3,3)]);
    switch(convcase)
        case 1
            quat(2) = (R(1,1) - (R(2,2) + R(3,3))) + 1;
            quat(3) = (R(1,2) + R(2,1));
            quat(4) = (R(3,1) + R(1,3));
            quat(1) = (R(3,2) - R(2,3));
        case 2
            quat(3) = (R(2,2) - (R(3,3) + R(1,1))) + 1;
            quat(4) = (R(2,3) + R(3,2));
            quat(2) = (R(1,2) + R(2,1));
            quat(1) = (R(1,3) - R(3,1));
        case 3
            quat(4) = (R(3,3) - (R(1,1) + R(2,2))) + 1;
            quat(2) = (R(3,1) + R(1,3));
            quat(3) = (R(2,3) + R(3,2));
            quat(1) = (R(2,1) - R(1,2));
    end
end

quat = quat/norm(quat);

% make sure cos component of quaternion is always positive (saves headaches later when doing nearest neighbors)
if( quat(1) < 0 )
    quat = -quat;
end
