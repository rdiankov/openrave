%% Script for simulating a camera and displaying its results in real-time
function testcamerasensor(render)

more off; % turn off output paging
addopenravepaths()

if( ~exist('render','var') )
    render = [0];
end

orEnvLoadScene('data/testwamcamera.env.xml',1); % reset the scene
robots = orEnvGetRobots();
robotid = robots{1}.id;

% to turn on the rendering, send a command to the sensor
for i = 1:length(render)
    orRobotSensorConfigure(robotid, render(i), 'PowerOn')
    orRobotSensorConfigure(robotid, render(i), 'RenderDataOn')
end

sensorindex = 0;

while(1)
    % get image
    tic;
    data = orRobotSensorGetData(robotid, sensorindex);
    toc;
    imshow(data.I*64);
    pause(1);
end
