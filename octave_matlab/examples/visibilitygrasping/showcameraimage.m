% I=showcameraimage(robotid)

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
function I = showcameraimage(robotid)
orRobotSensorSend(robotid,0,'power','1');
pause(0.3);
data = orRobotSensorGetData(robotid, 0);
orRobotSensorSend(robotid,0,'power','0');
I = im2double(data.I);
imshow(I*64);
