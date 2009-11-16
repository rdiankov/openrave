% showcameraimage(robotid)
function I = showcameraimage(robotid)
orRobotSensorSend(robotid,0,'power','1');
pause(0.3);
data = orRobotSensorGetData(robotid, 0);
orRobotSensorSend(robotid,0,'power','0');
I = im2double(data.I);
imshow(I*64);
